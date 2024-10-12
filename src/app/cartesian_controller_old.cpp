#include "app/cartesian_controller_old.h"
#include "app/common.h"
#include "app/config.h"
#include "utils.h"
#include <stdexcept>
#include <sys/syscall.h>
#include <sys/types.h>

namespace arx
{

Arx5CartesianController::Arx5CartesianController(RobotConfig robot_config, ControllerConfig controller_config,
                                                 std::string interface_name, std::string urdf_path)
    : _can_handle(interface_name),
      _logger(spdlog::stdout_color_mt(robot_config.robot_model + std::string("_") + interface_name)),
      _robot_config(robot_config), _controller_config(controller_config)
{
    _logger->set_pattern("[%H:%M:%S %n %^%l%$] %v");
    _solver = std::make_shared<Arx5Solver>(urdf_path, _robot_config.joint_dof, _robot_config.base_link_name,
                                           _robot_config.eef_link_name, _robot_config.gravity_vector);
    _init_robot();
    _background_send_recv_thread = std::thread(&Arx5CartesianController::_background_send_recv, this);
    _logger->info("Background send_recv task is running at ID: {}", syscall(SYS_gettid));
}

Arx5CartesianController::Arx5CartesianController(std::string model, std::string interface_name, std::string urdf_path)
    : Arx5CartesianController::Arx5CartesianController(
          RobotConfigFactory::get_instance().get_config(model),
          ControllerConfigFactory::get_instance().get_config(
              "cartesian_controller", RobotConfigFactory::get_instance().get_config(model).joint_dof),
          interface_name, urdf_path)
{
}

Arx5CartesianController::~Arx5CartesianController()
{
    Gain damping_gain{_robot_config.joint_dof};
    damping_gain.kd = _controller_config.default_kd;
    // damping_gain.kd[0] *= 3;
    // damping_gain.kd[1] *= 3;
    // damping_gain.kd[2] *= 3;
    // damping_gain.kd[3] *= 1.5;
    _logger->info("Set to damping before exit");
    set_gain(damping_gain);
    _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
    _input_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
    _enable_gravity_compensation = false;
    sleep_ms(2000);
    _destroy_background_threads = true;
    _background_send_recv_thread.join();
    _logger->info("background send_recv task joined");
    spdlog::drop(_logger->name());
    _logger.reset();
    _solver.reset();
}

void Arx5CartesianController::_init_robot()
{
    for (int i = 0; i < _robot_config.joint_dof; ++i)
    {
        if (_robot_config.motor_type[i] == MotorType::DM_J4310 || _robot_config.motor_type[i] == MotorType::DM_J4340)
        {
            int id = _robot_config.motor_id[i];
            _can_handle.enable_DM_motor(id);
            sleep_us(1000);
        }
    }
    if (_robot_config.gripper_motor_type == MotorType::DM_J4310)
    {
        _can_handle.enable_DM_motor(_robot_config.gripper_motor_id);
        sleep_us(1000);
    }

    Gain gain{_robot_config.joint_dof};
    gain.kd = _controller_config.default_kd;
    _input_joint_cmd = JointState(_robot_config.joint_dof); // initialize joint command to zero

    set_gain(gain); // set to damping by default
    for (int i = 0; i <= 10; ++i)
    {
        // make sure all the motor positions are updated
        _send_recv();
        sleep_ms(5);
    }
    // Check whether any motor has non-zero position
    if (_joint_state.pos == VecDoF::Zero(_robot_config.joint_dof))
    {
        _logger->error("None of the motors are initialized. Please check the connection or power of the arm.");
        throw std::runtime_error(
            "None of the motors are initialized. Please check the connection or power of the arm.");
    }
    _input_eef_cmd = get_eef_state();
    _output_eef_cmd = get_eef_state();
    _interp_start_eef_cmd = get_eef_state();
    _input_eef_cmd.timestamp = 0;
    _output_eef_cmd.timestamp = 0;
    _interp_start_eef_cmd.timestamp = 0;

    _background_send_recv_running = true;
}

void Arx5CartesianController::set_eef_cmd(EEFState new_cmd)
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    if (new_cmd.gripper_vel != 0 || new_cmd.gripper_torque != 0)
    {
        _logger->warn("Gripper vel and torque control is not supported yet.");
        new_cmd.gripper_vel = 0;
        new_cmd.gripper_torque = 0;
    }
    if (new_cmd.timestamp != 0 && new_cmd.timestamp < get_timestamp())
    {
        _logger->warn("EEF command timestamp ({:.4f}s) is not 0 but in the past (current timestamp: {:.4f}s). New EEF "
                      "command is ignored.",
                      new_cmd.timestamp, get_timestamp());
        return;
    }
    _input_eef_cmd = new_cmd;
    _interp_start_eef_cmd = _output_eef_cmd;
}

std::tuple<EEFState, EEFState> Arx5CartesianController::get_eef_cmd()
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    return std::make_tuple(_input_eef_cmd, _output_eef_cmd);
}

EEFState Arx5CartesianController::get_eef_state()
{
    std::lock_guard<std::mutex> lock(_state_mutex);
    EEFState eef_state;
    eef_state.timestamp = _joint_state.timestamp;
    eef_state.pose_6d = _solver->forward_kinematics(_joint_state.pos);
    eef_state.gripper_pos = _joint_state.gripper_pos;
    eef_state.gripper_vel = _joint_state.gripper_vel;
    eef_state.gripper_torque = _joint_state.gripper_torque;
    return eef_state;
}

JointState Arx5CartesianController::get_joint_state()
{
    std::lock_guard<std::mutex> lock(_state_mutex);
    return _joint_state;
}

std::tuple<JointState, JointState> Arx5CartesianController::get_joint_cmd()
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    return std::make_tuple(_input_joint_cmd, _output_joint_cmd);
}

double Arx5CartesianController::get_timestamp()
{
    return double(get_time_us() - _start_time_us) / 1e6;
}

Gain Arx5CartesianController::get_gain()
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    return _gain;
}

void Arx5CartesianController::set_gain(Gain new_gain)
{
    // Make sure the robot doesn't jump when setting kp to non-zero
    if (_gain.kp.isZero() && !new_gain.kp.isZero())
    {
        double max_pos_error = (_joint_state.pos - _output_joint_cmd.pos).cwiseAbs().maxCoeff();
        double pos_error_threshold = 0.2;
        double kp_threshold = 1;
        if (max_pos_error > pos_error_threshold && new_gain.kp.maxCoeff() > kp_threshold)
        {
            _logger->error("Cannot set kp too large when the joint pos cmd is far from current pos.");
            _logger->error(
                "Target max kp: {}, kp threshold: {}. Current pos: {}, cmd pos: {}, position error threshold: {}",
                new_gain.kp.maxCoeff(), kp_threshold, vec2str(_joint_state.pos), vec2str(_output_joint_cmd.pos),
                pos_error_threshold);
            _background_send_recv_running = false;
            throw std::runtime_error("Cannot set kp to non-zero when the joint pos cmd is far from current pos.");
        }
    }
    _gain = new_gain;
}

RobotConfig Arx5CartesianController::get_robot_config()
{
    return _robot_config;
}

ControllerConfig Arx5CartesianController::get_controller_config()
{
    return _controller_config;
}

Pose6d Arx5CartesianController::get_home_pose()
{
    return _solver->forward_kinematics(VecDoF::Zero(_robot_config.joint_dof));
}

void Arx5CartesianController::reset_to_home()
{
    JointState joint_cmd{_robot_config.joint_dof};
    EEFState eef_cmd;
    Gain gain{_robot_config.joint_dof};
    JointState init_state = get_joint_state();
    Gain init_gain = get_gain();
    Gain target_gain{_robot_config.joint_dof};
    if (init_gain.kp.isZero())
    {
        _logger->info("Current kp is zero. Setting to default kp kd");
        target_gain = Gain(_controller_config.default_kp, _controller_config.default_kd,
                           _controller_config.default_gripper_kp, _controller_config.default_gripper_kd);
    }
    else
    {
        target_gain = init_gain;
    }

    JointState target_state{_robot_config.joint_dof};

    // calculate the maximum joint position error
    double max_pos_error = (init_state.pos - VecDoF::Zero(_robot_config.joint_dof)).cwiseAbs().maxCoeff();
    max_pos_error = std::max(max_pos_error, init_state.gripper_pos * 2 / _robot_config.gripper_width);
    // interpolate from current kp kd to default kp kd in max(max_pos_error*2, 0.5)s
    // and keep the target for 0.5s
    double step_num = std::max(max_pos_error * 2, 0.5) / _controller_config.controller_dt;
    _logger->info("Start reset to home in {:.3f}s, max_pos_error: {:.3f}",
                  std::max(max_pos_error * 2, double(0.5)) + 0.5, max_pos_error);

    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = true;
    for (int i = 0; i <= step_num; ++i)
    {
        double alpha = double(i) / step_num;
        gain = init_gain * (1 - alpha) + target_gain * alpha;
        joint_cmd = init_state * (1 - alpha) + target_state * alpha;
        set_gain(gain);
        eef_cmd.pose_6d = _solver->forward_kinematics(joint_cmd.pos);
        eef_cmd.gripper_pos = joint_cmd.gripper_pos;
        set_eef_cmd(eef_cmd);
        sleep_ms(5);
    }
    sleep_ms(500);
    _logger->info("Finish reset to home");
    _background_send_recv_running = prev_running;
}

void Arx5CartesianController::set_to_damping()
{
    JointState joint_cmd{_robot_config.joint_dof};
    EEFState eef_cmd;
    JointState joint_state{_robot_config.joint_dof};
    Gain gain{_robot_config.joint_dof};
    JointState init_state = get_joint_state();
    Gain init_gain = get_gain();
    Gain target_gain{_robot_config.joint_dof};
    target_gain.kd = _controller_config.default_kd;
    _logger->info("Start set to damping");

    joint_state = get_joint_state();
    eef_cmd.pose_6d = _solver->forward_kinematics(joint_state.pos);
    eef_cmd.gripper_pos = joint_state.gripper_pos;
    set_gain(target_gain);
    set_eef_cmd(eef_cmd);

    sleep_ms(500);
    _logger->info("Finish set to damping");
}

void Arx5CartesianController::set_log_level(spdlog::level::level_enum log_level)
{
    _logger->set_level(log_level);
}

void Arx5CartesianController::_update_output_cmd()
{
    std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);

    JointState prev_output_cmd = _output_joint_cmd;

    _output_joint_cmd = _input_joint_cmd;

    // Joint velocity clipping
    double dt = _controller_config.controller_dt;
    for (int i = 0; i < 6; ++i)
    {
        if (_gain.kp[i] > 0)
        {
            double delta_pos = _input_joint_cmd.pos[i] - prev_output_cmd.pos[i];
            double max_vel = _robot_config.joint_vel_max[i];
            if (std::abs(delta_pos) > max_vel * dt)
            {
                _output_joint_cmd.pos[i] = prev_output_cmd.pos[i] + max_vel * dt * delta_pos / std::abs(delta_pos);
                _logger->debug("Joint {} pos {:.3f} pos cmd clipped: {:.3f} to {:.3f}", i, _joint_state.pos[i],
                               _input_joint_cmd.pos[i], _output_joint_cmd.pos[i]);
            }
        }
        else
        {
            _output_joint_cmd.pos[i] = _joint_state.pos[i];
        }
        if (_gain.gripper_kp > 0)
        {
            double gripper_delta_pos = _input_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos;
            if (std::abs(gripper_delta_pos) / dt > _robot_config.gripper_vel_max)
            {
                _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos + _robot_config.gripper_vel_max * dt *
                                                                                  gripper_delta_pos /
                                                                                  std::abs(gripper_delta_pos);
                if (std::abs(_input_joint_cmd.gripper_pos - _output_joint_cmd.gripper_pos) >= 0.001)
                    _logger->debug("Gripper pos cmd clipped: {:.3f} to {:.3f}", _input_joint_cmd.gripper_pos,
                                   _output_joint_cmd.gripper_pos);
            }
        }
        else
        {
            _output_joint_cmd.gripper_pos = _joint_state.gripper_pos;
        }
    }

    // Joint pos clipping
    for (int i = 0; i < 6; ++i)
    {
        if (_output_joint_cmd.pos[i] < _robot_config.joint_pos_min[i])
        {
            _logger->debug("Joint {} pos {:.3f} pos cmd clipped from {:.3f} to min {:.3f}", i, _joint_state.pos[i],
                           _output_joint_cmd.pos[i], _robot_config.joint_pos_min[i]);
            _output_joint_cmd.pos[i] = _robot_config.joint_pos_min[i];
        }
        else if (_output_joint_cmd.pos[i] > _robot_config.joint_pos_max[i])
        {
            _logger->debug("Joint {} pos {:.3f} pos cmd clipped from {:.3f} to max {:.3f}", i, _joint_state.pos[i],
                           _output_joint_cmd.pos[i], _robot_config.joint_pos_max[i]);
            _output_joint_cmd.pos[i] = _robot_config.joint_pos_max[i];
        }
    }
    // Gripper pos clipping
    if (_output_joint_cmd.gripper_pos < 0)
    {
        if (_output_joint_cmd.gripper_pos < -0.005)
            _logger->debug("Gripper pos cmd clipped from {:.3f} to min: {:.3f}", _output_joint_cmd.gripper_pos, 0.0);
        _output_joint_cmd.gripper_pos = 0;
    }
    else if (_output_joint_cmd.gripper_pos > _robot_config.gripper_width)
    {
        if (_output_joint_cmd.gripper_pos > _robot_config.gripper_width + 0.005)
            _logger->debug("Gripper pos cmd clipped from {:.3f} to max: {:.3f}", _output_joint_cmd.gripper_pos,
                           _robot_config.gripper_width);
        _output_joint_cmd.gripper_pos = _robot_config.gripper_width;
    }
    if (std::abs(_joint_state.gripper_torque) > _robot_config.gripper_torque_max / 2)
    {
        double sign = _joint_state.gripper_torque > 0 ? 1 : -1;
        // -1 for closing blocked, 1 for opening blocked
        double delta_pos = _output_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos;
        // negative for closing, positive for opening
        if (delta_pos * sign > 0)
        {
            _logger->debug("Gripper torque is too large, gripper pos cmd is not updated");
            _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos;
        }
    }

    // Torque clipping
    for (int i = 0; i < 6; ++i)
    {
        if (_output_joint_cmd.torque[i] > _robot_config.joint_torque_max[i])
        {
            _logger->debug("Joint {} torque cmd clipped from {:.3f} to max {:.3f}", i, _output_joint_cmd.torque[i],
                           _robot_config.joint_torque_max[i]);
            _output_joint_cmd.torque[i] = _robot_config.joint_torque_max[i];
        }
        else if (_output_joint_cmd.torque[i] < -_robot_config.joint_torque_max[i])
        {
            _logger->debug("Joint {} torque cmd clipped from {:.3f} to min {:.3f}", i, _output_joint_cmd.torque[i],
                           -_robot_config.joint_torque_max[i]);
            _output_joint_cmd.torque[i] = -_robot_config.joint_torque_max[i];
        }
    }
}

void Arx5CartesianController::_over_current_protection()
{
    bool over_current = false;
    for (int i = 0; i < 6; ++i)
    {
        if (std::abs(_joint_state.torque[i]) > _robot_config.joint_torque_max[i])
        {
            over_current = true;
            _logger->error("Over current detected once on joint {}, current: {:.3f}", i, _joint_state.torque[i]);
            break;
        }
    }
    if (std::abs(_joint_state.gripper_torque) > _robot_config.gripper_torque_max)
    {
        over_current = true;
        _logger->error("Over current detected once on gripper, current: {:.3f}", _joint_state.gripper_torque);
    }
    if (over_current)
    {
        _over_current_cnt++;
        if (_over_current_cnt > _controller_config.over_current_cnt_max)
        {
            _logger->error("Over current detected, robot is set to damping. Please restart the program.");
            _enter_emergency_state();
        }
    }
    else
    {
        _over_current_cnt = 0;
    }
}

void Arx5CartesianController::_check_joint_state_sanity()
{
    for (int i = 0; i < 6; ++i)
    {
        if (std::abs(_joint_state.pos[i]) > _robot_config.joint_pos_max[i] + 3.14 ||
            std::abs(_joint_state.pos[i]) < _robot_config.joint_pos_min[i] - 3.14)
        {
            _logger->error("Joint {} pos data error: {:.3f}. Please restart the program.", i, _joint_state.pos[i]);
            _enter_emergency_state();
        }
        if (std::abs(_input_joint_cmd.pos[i]) > _robot_config.joint_pos_max[i] + 3.14 ||
            std::abs(_input_joint_cmd.pos[i]) < _robot_config.joint_pos_min[i] - 3.14)
        {
            _logger->error("Joint {} command data error: {:.3f}. Please restart the program.", i,
                           _input_joint_cmd.pos[i]);
            _enter_emergency_state();
        }
        if (std::abs(_joint_state.torque[i]) > 100 * _robot_config.joint_torque_max[i])
        {
            _logger->error("Joint {} torque data error: {:.3f}. Please restart the program.", i,
                           _joint_state.torque[i]);
            _enter_emergency_state();
        }
    }
    // Gripper should be around 0~_robot_config.gripper_width
    double gripper_width_tolerance = 0.005; // m
    if (_joint_state.gripper_pos < -gripper_width_tolerance ||
        _joint_state.gripper_pos > _robot_config.gripper_width + gripper_width_tolerance)
    {
        _logger->error("Gripper position error: got {:.3f} but should be in 0~{:.3f} (m). Please close the gripper "
                       "before turning the arm on or recalibrate gripper home and width.",
                       _joint_state.gripper_pos, _robot_config.gripper_width);
        _enter_emergency_state();
    }
}

void Arx5CartesianController::_enter_emergency_state()
{
    Gain damping_gain{_robot_config.joint_dof};
    damping_gain.kd = _controller_config.default_kd;
    damping_gain.kd[1] *= 3;
    damping_gain.kd[2] *= 3;
    damping_gain.kd[3] *= 1.5;
    set_gain(damping_gain);
    _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
    _input_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
    _logger->error("Emergency state entered. Please restart the program.");
    while (true)
    {
        std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);
        set_gain(damping_gain);
        _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
        _input_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
        _send_recv();
        sleep_ms(5);
    }
}
bool Arx5CartesianController::_send_recv()
{
    // TODO: in the motor documentation, there shouldn't be these torque constants. Torque will go directly into the
    // motors
    const double torque_constant_EC_A4310 = 1.4; // Nm/A
    const double torque_constant_DM_J4310 = 0.424;
    const double torque_constant_DM_J4340 = 1.0;
    int start_time_us = get_time_us();

    _update_output_cmd();
    int update_cmd_time_us = get_time_us();
    int communicate_sleep_us = 150;

    for (int i = 0; i < _robot_config.joint_dof; i++)
    {
        int start_send_motor_time_us = get_time_us();
        if (_robot_config.motor_type[i] == MotorType::EC_A4310)
        {
            _can_handle.send_EC_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i], _output_joint_cmd.pos[i],
                                          _output_joint_cmd.vel[i],
                                          _output_joint_cmd.torque[i] / torque_constant_EC_A4310);
        }
        else if (_robot_config.motor_type[i] == MotorType::DM_J4310)
        {

            _can_handle.send_DM_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i], _output_joint_cmd.pos[i],
                                          _output_joint_cmd.vel[i],
                                          _output_joint_cmd.torque[i] / torque_constant_DM_J4310);
        }
        else if (_robot_config.motor_type[i] == MotorType::DM_J4340)
        {
            _can_handle.send_DM_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i], _output_joint_cmd.pos[i],
                                          _output_joint_cmd.vel[i],
                                          _output_joint_cmd.torque[i] / torque_constant_DM_J4340);
        }
        else
        {
            _logger->error("Motor type not supported.");
            return false;
        }
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }

    // Send gripper command (gripper is using DM motor)
    if (_robot_config.gripper_motor_type == MotorType::DM_J4310)
    {
        int start_send_motor_time_us = get_time_us();

        double gripper_motor_pos =
            _output_joint_cmd.gripper_pos / _robot_config.gripper_width * _robot_config.gripper_open_readout;
        _can_handle.send_DM_motor_cmd(_robot_config.gripper_motor_id, _gain.gripper_kp, _gain.gripper_kd,
                                      gripper_motor_pos, 0, 0);
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }

    int start_get_motor_msg_time_us = get_time_us();
    std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
    int get_motor_msg_time_us = get_time_us();

    // _logger->trace("update_cmd: {} us, send_motor_0: {} us, send_motor_1: {} us, send_motor_2: {} us, send_motor_3:
    // {} us, send_motor_4: {} us, send_motor_5: {} us, send_motor_6: {} us, get_motor_msg: {} us",
    //                update_cmd_time_us - start_time_us, send_motor_0_time_us - start_send_motor_0_time_us,
    //                send_motor_1_time_us - start_send_motor_1_time_us, send_motor_2_time_us -
    //                start_send_motor_2_time_us, send_motor_3_time_us - start_send_motor_3_time_us,
    //                send_motor_4_time_us - start_send_motor_4_time_us, send_motor_5_time_us -
    //                start_send_motor_5_time_us, send_motor_6_time_us - start_send_motor_6_time_us,
    //                get_motor_msg_time_us - start_get_motor_msg_time_us);

    std::lock_guard<std::mutex> guard_state(_state_mutex);
    for (int i = 0; i < _robot_config.joint_dof; i++)
    {
        _joint_state.pos[i] = motor_msg[_robot_config.motor_id[i]].angle_actual_rad;
        _joint_state.vel[i] = motor_msg[_robot_config.motor_id[i]].speed_actual_rad;

        // Torque: matching the values (there must be something wrong)
        if (_robot_config.motor_type[i] == MotorType::EC_A4310)
        {
            _joint_state.torque[i] = motor_msg[_robot_config.motor_id[i]].current_actual_float *
                                     torque_constant_EC_A4310 * torque_constant_EC_A4310;
            // Why there are two torque_constant_EC_A4310?
        }
        else if (_robot_config.motor_type[i] == MotorType::DM_J4310)
        {
            _joint_state.torque[i] =
                motor_msg[_robot_config.motor_id[i]].current_actual_float * torque_constant_DM_J4310;
        }
        else if (_robot_config.motor_type[i] == MotorType::DM_J4340)
        {
            _joint_state.torque[i] =
                motor_msg[_robot_config.motor_id[i]].current_actual_float * torque_constant_DM_J4340;
        }
    }

    _joint_state.gripper_pos = motor_msg[_robot_config.gripper_motor_id].angle_actual_rad /
                               _robot_config.gripper_open_readout * _robot_config.gripper_width;

    _joint_state.gripper_vel = motor_msg[_robot_config.gripper_motor_id].speed_actual_rad /
                               _robot_config.gripper_open_readout * _robot_config.gripper_width;

    _joint_state.gripper_torque =
        motor_msg[_robot_config.gripper_motor_id].current_actual_float * torque_constant_DM_J4310;
    _joint_state.timestamp = get_timestamp();
    return true;
}

void Arx5CartesianController::_calc_joint_cmd()
{
    JointState joint_cmd{_robot_config.joint_dof};
    JointState joint_state = get_joint_state();
    std::tuple<bool, VecDoF> ik_results;

    {
        std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);
        // Calculate output eef command (according to the interpolation)
        if (_input_eef_cmd.timestamp == 0) // No interpolation. Directly update the target
        {
            _output_eef_cmd = _input_eef_cmd;
            _output_eef_cmd.timestamp = get_timestamp();
        }
        else // Interpolate the current timestamp between _interp_start_eef_cmd and _input_eef_cmd
        {
            // _logger->debug("Received non-zero eef_cmd!");
            double current_timestamp = get_timestamp();
            assert(current_timestamp >= _interp_start_eef_cmd.timestamp);
            assert(_input_eef_cmd.timestamp > _interp_start_eef_cmd.timestamp);
            if (current_timestamp > _input_eef_cmd.timestamp)
            // Current timestamp has already exceed the interpolation target: hold at this target pose
            {
                _output_eef_cmd = _input_eef_cmd;
                _output_eef_cmd.timestamp = current_timestamp;
            }
            else // Apply interpolation
            {
                double alpha = (current_timestamp - _interp_start_eef_cmd.timestamp) /
                               (_input_eef_cmd.timestamp - _interp_start_eef_cmd.timestamp);
                assert(alpha >= 0 && alpha <= 1);
                _output_eef_cmd.pose_6d = _interp_start_eef_cmd.pose_6d * (1 - alpha) + _input_eef_cmd.pose_6d * alpha;
                _output_eef_cmd.gripper_pos =
                    _interp_start_eef_cmd.gripper_pos * (1 - alpha) + _input_eef_cmd.gripper_pos * alpha;
                _output_eef_cmd.timestamp = current_timestamp;
            }
        }

        if (_output_eef_cmd.pose_6d.isZero() || _output_eef_cmd.pose_6d.norm() < 0.01)
        {
            _logger->error("EEF command should not be set close to zero. To start from the home pose, please call "
                           "get_home_pose().");
            _enter_emergency_state();
        }
        ik_results = _solver->inverse_kinematics(_output_eef_cmd.pose_6d, joint_state.pos);
        joint_cmd.gripper_pos = _output_eef_cmd.gripper_pos;
    }
    bool success = std::get<0>(ik_results);
    VecDoF joint_pos = std::get<1>(ik_results);

    VecDoF clipped_joint_pos = joint_pos.cwiseMax(_robot_config.joint_pos_min).cwiseMin(_robot_config.joint_pos_max);

    if (success)
    {
        joint_cmd.pos = _joint_pos_filter.filter(clipped_joint_pos);
        if (_enable_gravity_compensation)
        {
            // Use the torque of the current joint positions
            VecDoF joint_torque = _solver->inverse_dynamics(_joint_state.pos, VecDoF::Zero(_robot_config.joint_dof),
                                                            VecDoF::Zero(_robot_config.joint_dof));
            joint_cmd.torque = _joint_torque_filter.filter(joint_torque);
        }
        _input_joint_cmd = joint_cmd;
    }
}

void Arx5CartesianController::_background_send_recv()
{
    while (!_destroy_background_threads)
    {
        int start_time_us = get_time_us();
        if (_background_send_recv_running)
        {
            _over_current_protection();
            _check_joint_state_sanity();
            _calc_joint_cmd();
            _send_recv();
        }
        int elapsed_time_us = get_time_us() - start_time_us;
        int sleep_time_us = int(_controller_config.controller_dt * 1e6) - elapsed_time_us;
        if (sleep_time_us > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
        }
        else if (sleep_time_us < -500)
        {
            _logger->debug("Background send_recv task is running too slow, time: {} us", elapsed_time_us);
        }
    }
}
} // namespace arx