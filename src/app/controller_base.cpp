#include "app/controller_base.h"
#include "app/common.h"
#include "utils.h"
#include <array>
#include <stdexcept>
#include <sys/syscall.h>
#include <sys/types.h>
using namespace arx;

Arx5ControllerBase::Arx5ControllerBase(RobotConfig robot_config, ControllerConfig controller_config,
                                       std::string interface_name, std::string urdf_path)
    : _can_handle(interface_name),
      _logger(spdlog::stdout_color_mt(robot_config.robot_model + std::string("_") + interface_name)),
      _robot_config(robot_config), _controller_config(controller_config)
{
    _logger->info("Start initializing solver");
    _logger->set_pattern("[%H:%M:%S %n %^%l%$] %v");
    _solver = std::make_shared<Arx5Solver>(urdf_path, _robot_config.joint_dof, _robot_config.joint_pos_min,
                                           _robot_config.joint_pos_max, _robot_config.base_link_name,
                                           _robot_config.eef_link_name, _robot_config.gravity_vector);
    _logger->info("Done initializing solver");
    if (_robot_config.robot_model == "X5" && !_controller_config.shutdown_to_passive)
    {
        _logger->warn("When shutting down X5 robot arms, the motors have to be set to passive. "
                      "_controller_config.shutdown_to_passive is set to `true`");
        _controller_config.shutdown_to_passive = true;
    }
    _init_robot();
    _background_send_recv_thread = std::thread(&Arx5ControllerBase::_background_send_recv, this);
    _background_send_recv_running = _controller_config.background_send_recv;
    _logger->info("Background send_recv task is running at ID: {}", syscall(SYS_gettid));
}

Arx5ControllerBase::~Arx5ControllerBase()
{
    if (_controller_config.shutdown_to_passive)
    {
        _logger->info("Set to damping before exit");
        Gain damping_gain{_robot_config.joint_dof};
        damping_gain.kd = _controller_config.default_kd;

        // Increase damping if needed
        // damping_gain.kd[0] *= 3;
        // damping_gain.kd[1] *= 3;
        // damping_gain.kd[2] *= 3;
        // damping_gain.kd[3] *= 1.5;

        set_gain(damping_gain);
        {
            std::lock_guard<std::mutex> guard(_cmd_mutex);
            _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
            _input_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
            _input_joint_cmd.timestamp = 0;
            _output_joint_cmd = _input_joint_cmd;
            _intermediate_joint_cmd = _input_joint_cmd;
        }
        _background_send_recv_running = true;
        _controller_config.gravity_compensation = false;
        sleep_ms(2000);
    }
    else
    {
        _logger->info("Disconnect motors without setting to damping");
    }

    _destroy_background_threads = true;
    _background_send_recv_thread.join();
    _logger->info("background send_recv task joined");
    spdlog::drop(_logger->name());
    _logger.reset();
    _solver.reset();
}

std::tuple<JointState, JointState> Arx5ControllerBase::get_joint_cmd()
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    return std::make_tuple(_input_joint_cmd, _output_joint_cmd);
}

JointState Arx5ControllerBase::get_joint_state()
{
    std::lock_guard<std::mutex> guard(_state_mutex);
    return _joint_state;
}

EEFState Arx5ControllerBase::get_eef_state()
{
    EEFState eef_state;
    JointState joint_state = get_joint_state();
    Pose6d tool_pose = _solver->forward_kinematics(joint_state.pos);
    eef_state.pose_6d = tool_pose;
    eef_state.timestamp = joint_state.timestamp;
    eef_state.gripper_pos = joint_state.gripper_pos;
    eef_state.gripper_vel = joint_state.gripper_vel;
    eef_state.gripper_torque = joint_state.gripper_torque;
    return eef_state;
}

void Arx5ControllerBase::set_gain(Gain new_gain)
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
    {
        std::lock_guard<std::mutex> guard(_cmd_mutex);
        _gain = new_gain;
    }
}

Gain Arx5ControllerBase::get_gain()
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    return _gain;
}

double Arx5ControllerBase::get_timestamp()
{
    return double(get_time_us() - _start_time_us) / 1e6;
}
RobotConfig Arx5ControllerBase::get_robot_config()
{
    return _robot_config;
}
ControllerConfig Arx5ControllerBase::get_controller_config()
{
    return _controller_config;
}
void Arx5ControllerBase::set_log_level(spdlog::level::level_enum level)
{
    _logger->set_level(level);
}

void Arx5ControllerBase::reset_to_home()
{
    JointState init_state = get_joint_state();
    Gain init_gain = get_gain();
    double init_gripper_kp = _gain.gripper_kp;
    double init_gripper_kd = _gain.gripper_kd;
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
    // interpolate from current kp kd to default kp kd in max(max_pos_error, 0.5)s
    // and keep the target for max(max_pos_error, 0.5)s
    double wait_time = std::max(max_pos_error, 0.5);
    int step_num = int(wait_time / _controller_config.controller_dt);
    _logger->info("Start reset to home in {:.3f}s, max_pos_error: {:.3f}", std::max(max_pos_error, double(0.5)) + 0.5,
                  max_pos_error);

    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = true;
    target_state.timestamp = get_timestamp() + wait_time;
    {
        std::lock_guard<std::mutex> lock(_interpolator_mutex);
        _interpolator.update(get_timestamp(), target_state);
    }
    Gain new_gain{_robot_config.joint_dof};
    for (int i = 0; i <= step_num; i++)
    {
        double alpha = double(i) / step_num;
        new_gain = init_gain * (1 - alpha) + target_gain * alpha;
        set_gain(new_gain);
        sleep_us(int(_controller_config.controller_dt * 1e6));
    }

    sleep_ms(500);
    _logger->info("Finish reset to home");
    _background_send_recv_running = prev_running;
}

void Arx5ControllerBase::set_to_damping()
{
    Gain damping_gain{_robot_config.joint_dof};
    damping_gain.kd = _controller_config.default_kd;
    set_gain(damping_gain);
    sleep_ms(10);
    JointState joint_state = get_joint_state();
    {
        std::lock_guard<std::mutex> lock(_interpolator_mutex);
        joint_state.vel = VecDoF::Zero(_robot_config.joint_dof);
        joint_state.torque = VecDoF::Zero(_robot_config.joint_dof);
        _interpolator.init_fixed(joint_state);
    }
}

// ---------------------- Private functions ----------------------

void Arx5ControllerBase::_init_robot()
{
    // Background send receive is disabled during initialization
    int init_rounds = 10; // Make sure the states of each motor is fully initialized
    _logger->info("Enter _init_robot");
    for (int j = 0; j < init_rounds; j++)
    {
        _recv();
        _logger->info("Done receiving only");
        _check_joint_state_sanity();
        _logger->info("Done checking sanity");
        _over_current_protection();
        _logger->info("Done over current protection");
    }
    _logger->info("Done receiving only");

    Gain gain{_robot_config.joint_dof};
    gain.kd = _controller_config.default_kd;

    JointState init_joint_state = get_joint_state();
    init_joint_state.vel = VecDoF::Zero(_robot_config.joint_dof);
    init_joint_state.torque = VecDoF::Zero(_robot_config.joint_dof);
    {
        std::lock_guard<std::mutex> guard(_cmd_mutex);
        _input_joint_cmd = init_joint_state;
    }
    set_gain(gain); // set to damping by default

    // Check whether any motor has non-zero position
    if (_joint_state.pos == VecDoF::Zero(_robot_config.joint_dof))
    {
        _logger->error("None of the motors are initialized. Please check the connection or power of the arm.");
        throw std::runtime_error(
            "None of the motors are initialized. Please check the connection or power of the arm.");
    }
    _logger->info("Done setting gain");
    _input_joint_cmd = get_joint_state();
    _input_joint_cmd.torque = init_joint_state.torque;
    _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
    _input_joint_cmd.timestamp = 0;
    _output_joint_cmd = _input_joint_cmd;
    _intermediate_joint_cmd = _input_joint_cmd;

    {
        std::lock_guard<std::mutex> lock(_interpolator_mutex);
        _interpolator.init_fixed(init_joint_state);
    }

    for (int j = 0; j < init_rounds; j++)
    {
        _send_recv();
        _check_joint_state_sanity();
        _over_current_protection();
    }
    _logger->info("Robot initialized");
}

void Arx5ControllerBase::_check_joint_state_sanity()
{
    for (int i = 0; i < _robot_config.joint_dof; ++i)
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

void Arx5ControllerBase::_over_current_protection()
{
    bool over_current = false;
    for (int i = 0; i < _robot_config.joint_dof; ++i)
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
            _logger->error("Over current detected, robot is set to damping. Please restart the "
                           "program.");
            _enter_emergency_state();
        }
    }
    else
    {
        _over_current_cnt = 0;
    }
}

void Arx5ControllerBase::_enter_emergency_state()
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

void Arx5ControllerBase::_update_joint_state()
{
    // TODO: in the motor documentation, there shouldn't be these torque constants. Torque will go directly into the
    // motors
    const double torque_constant_EC_A4310 = 1.4; // Nm/A
    const double torque_constant_DM_J4310 = 0.424;
    const double torque_constant_DM_J4340 = 1.0;
    std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
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
}

void Arx5ControllerBase::_update_output_cmd()
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    JointState prev_output_cmd = _output_joint_cmd;

    // TODO: deal with non-zero velocity and torque for joint control
    double timestamp = get_timestamp();
    {
        std::lock_guard<std::mutex> lock(_interpolator_mutex);
        _output_joint_cmd = _interpolator.interpolate(timestamp);
    }

    if (_controller_config.gravity_compensation)
    {
        _output_joint_cmd.torque += _solver->inverse_dynamics(_joint_state.pos, VecDoF::Zero(_robot_config.joint_dof),
                                                              VecDoF::Zero(_robot_config.joint_dof));
    }
    // Joint velocity clipping
    double dt = _controller_config.controller_dt;
    for (int i = 0; i < _robot_config.joint_dof; ++i)
    {
        if (_gain.kp[i] > 0)
        {

            double delta_pos = _output_joint_cmd.pos[i] - prev_output_cmd.pos[i];
            double max_vel = _robot_config.joint_vel_max[i];
            if (std::abs(delta_pos) > max_vel * dt)
            {
                double new_pos = prev_output_cmd.pos[i] + max_vel * dt * delta_pos / std::abs(delta_pos);
                _logger->debug("Joint {} pos {:.3f} pos cmd clipped: {:.3f} to {:.3f}", i, _joint_state.pos[i],
                               _output_joint_cmd.pos[i], new_pos);
                _output_joint_cmd.pos[i] = new_pos;
            }
        }
        else
        {
            _output_joint_cmd.pos[i] = _joint_state.pos[i];
        }

        // Gripper pos clipping
        if (_gain.gripper_kp > 0)
        {
            double gripper_delta_pos = _output_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos;
            if (std::abs(gripper_delta_pos) / dt > _robot_config.gripper_vel_max)
            {
                double new_gripper_pos = prev_output_cmd.gripper_pos + _robot_config.gripper_vel_max * dt *
                                                                           gripper_delta_pos /
                                                                           std::abs(gripper_delta_pos);
                if (std::abs(_output_joint_cmd.gripper_pos - _output_joint_cmd.gripper_pos) >= 0.001)
                    _logger->debug("Gripper pos cmd clipped: {:.3f} to {:.3f}", _output_joint_cmd.gripper_pos,
                                   _output_joint_cmd.gripper_pos);
                _output_joint_cmd.gripper_pos = new_gripper_pos;
            }
        }
        else
        {
            _output_joint_cmd.gripper_pos = _joint_state.gripper_pos;
        }
    }

    // Joint pos clipping
    for (int i = 0; i < _robot_config.joint_dof; ++i)
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
        double sign = _joint_state.gripper_torque > 0 ? 1 : -1; // -1 for closing blocked, 1 for opening blocked
        double delta_pos =
            _output_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos; // negative for closing, positive for opening
        if (delta_pos * sign > 0)
        {
            if (_prev_gripper_updated)
                _logger->warn("Gripper torque is too large, gripper pos cmd is not updated");
            _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos;
            _prev_gripper_updated = false;
        }
        else
            _prev_gripper_updated = true;
    }

    // Torque clipping
    for (int i = 0; i < _robot_config.joint_dof; ++i)
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

void Arx5ControllerBase::_send_recv()
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
        {
            std::lock_guard<std::mutex> lock(_cmd_mutex);
            if (_robot_config.motor_type[i] == MotorType::EC_A4310)
            {
                _can_handle.send_EC_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i],
                                              _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
                                              _output_joint_cmd.torque[i] / torque_constant_EC_A4310);
            }
            else if (_robot_config.motor_type[i] == MotorType::DM_J4310)
            {

                _can_handle.send_DM_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i],
                                              _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
                                              _output_joint_cmd.torque[i] / torque_constant_DM_J4310);
            }
            else if (_robot_config.motor_type[i] == MotorType::DM_J4340)
            {
                _can_handle.send_DM_motor_cmd(_robot_config.motor_id[i], _gain.kp[i], _gain.kd[i],
                                              _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
                                              _output_joint_cmd.torque[i] / torque_constant_DM_J4340);
            }
            else
            {
                _logger->error("Motor type not supported.");
                return;
            }
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

    // _logger->trace("update_cmd: {} us, send_motor_0: {} us, send_motor_1: {} us, send_motor_2: {} us, send_motor_3:
    // {} us, send_motor_4: {} us, send_motor_5: {} us, send_motor_6: {} us, get_motor_msg: {} us",
    //                update_cmd_time_us - start_time_us, send_motor_0_time_us - start_send_motor_0_time_us,
    //                send_motor_1_time_us - start_send_motor_1_time_us, send_motor_2_time_us -
    //                start_send_motor_2_time_us, send_motor_3_time_us - start_send_motor_3_time_us,
    //                send_motor_4_time_us - start_send_motor_4_time_us, send_motor_5_time_us -
    //                start_send_motor_5_time_us, send_motor_6_time_us - start_send_motor_6_time_us,
    //                get_motor_msg_time_us - start_get_motor_msg_time_us);

    _update_joint_state();
}

void Arx5ControllerBase::_recv()
{
    _logger->info("Enter recv only");
    int communicate_sleep_us = 300;
    for (int i = 0; i < _robot_config.joint_dof; i++)
    {
        int start_send_motor_time_us = get_time_us();
        if (_robot_config.motor_type[i] == MotorType::EC_A4310)
        {
            _logger->error("EC_A4310 motor type is not supported yet.");
            assert(false);
        }
        else if (_robot_config.motor_type[i] == MotorType::DM_J4310 ||
                 _robot_config.motor_type[i] == MotorType::DM_J4340 ||
                 _robot_config.motor_type[i] == MotorType::DM_J8009)
        {
            // sleep_ms(1);
            _logger->info("start enable motor {}", _robot_config.motor_id[i]);
            _can_handle.enable_DM_motor(_robot_config.motor_id[i]);
            _logger->info("end enable motor {}", _robot_config.motor_id[i]);
        }
        else
        {
            _logger->error("Motor type not supported.");
            assert(false);
        }
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }
    _logger->info("Done enabling regular motors");
    if (_robot_config.gripper_motor_type == MotorType::DM_J4310)
    {
        int start_send_motor_time_us = get_time_us();
        _can_handle.enable_DM_motor(_robot_config.gripper_motor_id);
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }
    sleep_ms(1); // Wait until all the messages are updated
    _logger->info("done sending enable messages");
    _update_joint_state();
}

void Arx5ControllerBase::_background_send_recv()
{
    while (!_destroy_background_threads)
    {
        int start_time_us = get_time_us();
        if (_background_send_recv_running)
        {
            _over_current_protection();
            _check_joint_state_sanity();
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

Pose6d Arx5ControllerBase::get_home_pose()
{
    return _solver->forward_kinematics(VecDoF::Zero(_robot_config.joint_dof));
}
