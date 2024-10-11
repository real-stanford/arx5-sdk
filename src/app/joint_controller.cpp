#include "app/joint_controller.h"
#include "utils.h"
#include <array>
#include <stdexcept>
#include <sys/syscall.h>
#include <sys/types.h>
using namespace arx;

Arx5JointController::Arx5JointController(RobotConfig robot_config, ControllerConfig controller_config,
                                         std::string interface_name, std::string urdf_path)
    : _can_handle(interface_name),
      _logger(spdlog::stdout_color_mt(robot_config.robot_model + std::string("_") + interface_name)),
      _robot_config(robot_config), _controller_config(controller_config)
{
    _logger->set_pattern("[%H:%M:%S %n %^%l%$] %v");
    _solver = std::make_shared<Arx5Solver>(urdf_path, _robot_config.joint_dof, _robot_config.base_link_name,
                                           _robot_config.eef_link_name, _robot_config.gravity_vector);
    _init_robot();
    _background_send_recv_thread = std::thread(&Arx5JointController::_background_send_recv, this);
    _background_send_recv_running = _controller_config.background_send_recv;
    _logger->info("Background send_recv task is running at ID: {}", syscall(SYS_gettid));
}

Arx5JointController::Arx5JointController(std::string model, std::string interface_name, std::string urdf_path)
    : Arx5JointController::Arx5JointController(
          RobotConfigFactory::get_instance().get_config(model),
          ControllerConfigFactory::get_instance().get_config(
              "joint_controller", RobotConfigFactory::get_instance().get_config(model).joint_dof),
          interface_name, urdf_path)

{
}

Arx5JointController::~Arx5JointController()
{
    Gain damping_gain{_robot_config.joint_dof};
    damping_gain.kd = _controller_config.default_kd;
    // damping_gain.kd[0] *= 3;
    // damping_gain.kd[1] *= 3;
    // damping_gain.kd[2] *= 3;
    // damping_gain.kd[3] *= 1.5;
    _logger->info("Set to damping before exit");
    set_gain(damping_gain);
    set_joint_cmd(JointState(_robot_config.joint_dof));
    _background_send_recv_running = true;
    sleep_ms(1000);
    _controller_config.gravity_compensation = false;
    sleep_ms(1000);
    _destroy_background_threads = true;
    _background_send_recv_thread.join();
    _logger->info("background send_recv task joined");
    spdlog::drop(_logger->name());
    _logger.reset();
    _solver.reset();
}

void Arx5JointController::_init_robot()
{
    // Background send receive is disabled during initialization
    int init_rounds = 10; // Make sure the states of each motor is fully initialized
    for (int j = 0; j < init_rounds; j++)
    {
        recv_once();
        _check_joint_state_sanity();
        _over_current_protection();
    }

    Gain gain{_robot_config.joint_dof};
    gain.kd = _controller_config.default_kd;

    JointState init_joint_state = get_state();
    init_joint_state.vel = VecDoF::Zero(_robot_config.joint_dof);

    if (!_controller_config.gravity_compensation)
    {
        init_joint_state.torque = VecDoF::Zero(_robot_config.joint_dof);
    }
    set_joint_cmd(init_joint_state); // initialize joint command to zero

    set_gain(gain); // set to damping by default
    // Check whether any motor has non-zero position
    if (_joint_state.pos == VecDoF::Zero(_robot_config.joint_dof))
    {
        _logger->error("None of the motors are initialized. Please check the connection or power of the arm.");
        throw std::runtime_error(
            "None of the motors are initialized. Please check the connection or power of the arm.");
    }
    _input_joint_cmd = get_state();
    _input_joint_cmd.torque = init_joint_state.torque;
    _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
    _input_joint_cmd.timestamp = 0;
    _output_joint_cmd = _input_joint_cmd;
    _intermediate_joint_cmd = _input_joint_cmd;
    _interp_start_joint_cmd = _input_joint_cmd;
}

JointState Arx5JointController::get_state()
{
    std::lock_guard<std::mutex> guard(_state_mutex);
    return _joint_state;
}

Pose6d Arx5JointController::get_tool_pose()
{
    if (_solver == nullptr)
    {
        throw std::runtime_error("Solver is not initialized, cannot run forward kinematics.");
    }
    return _solver->forward_kinematics(_joint_state.pos);
}

RobotConfig Arx5JointController::get_robot_config()
{
    return _robot_config;
}

ControllerConfig Arx5JointController::get_controller_config()
{
    return _controller_config;
}

void Arx5JointController::send_recv_once()
{
    if (_background_send_recv_running)
    {
        _logger->warn("send_recv task is already running in background. send_recv_once is ignored.");
        return;
    }
    _check_joint_state_sanity();
    _over_current_protection();
    _send_recv();
}

void Arx5JointController::_update_output_cmd()
{
    std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);

    JointState prev_output_cmd = _output_joint_cmd;

    // Calculate output joint command (according to the interpolation)
    if (_input_joint_cmd.timestamp == 0) // No interpolation. Directly update the target
    {
        _output_joint_cmd = _input_joint_cmd;
        _output_joint_cmd.timestamp = get_timestamp();
    }
    else // Interpolate the current timestamp between _interp_start_joint_cmd and _input_joint_cmd
    {
        double current_timestamp = get_timestamp();
        assert(current_timestamp >= _interp_start_joint_cmd.timestamp);
        assert(_input_joint_cmd.timestamp > _interp_start_joint_cmd.timestamp);
        if (current_timestamp > _input_joint_cmd.timestamp)
        // Current timestamp has already exceed the interpolation target: hold at this target pose
        {
            _output_joint_cmd = _input_joint_cmd;
            _output_joint_cmd.timestamp = current_timestamp;
        }
        else // Apply interpolation
        {
            double alpha = (current_timestamp - _interp_start_joint_cmd.timestamp) /
                           (_input_joint_cmd.timestamp - _interp_start_joint_cmd.timestamp);
            assert(alpha >= 0 && alpha <= 1);
            _output_joint_cmd.pos = _interp_start_joint_cmd.pos * (1 - alpha) + _input_joint_cmd.pos * alpha;
            _output_joint_cmd.vel =
                _interp_start_joint_cmd.vel + alpha * (_input_joint_cmd.vel - _interp_start_joint_cmd.vel);
            _output_joint_cmd.torque =
                _interp_start_joint_cmd.torque + alpha * (_input_joint_cmd.torque - _interp_start_joint_cmd.torque);
            _output_joint_cmd.gripper_pos =
                _interp_start_joint_cmd.gripper_pos +
                alpha * (_input_joint_cmd.gripper_pos - _interp_start_joint_cmd.gripper_pos);
            _output_joint_cmd.timestamp = current_timestamp;
            // _logger->debug("alpha: {:.3f}, start_pos[0]: {:.3f}, cmd_pos[0]: {:.3f}, output_pos[0]: {:.3f}", alpha,
            //                _interp_start_joint_cmd.pos[0], _input_joint_cmd.pos[0], _output_joint_cmd.pos[0]);
        }
    }
    _intermediate_joint_cmd = _output_joint_cmd;
    if (_controller_config.gravity_compensation)
    {
        VecDoF gravity_torque = _solver->inverse_dynamics(_joint_state.pos, VecDoF::Zero(_robot_config.joint_dof),
                                                          VecDoF::Zero(_robot_config.joint_dof));
        _output_joint_cmd.torque += gravity_torque;
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
                _output_joint_cmd.pos[i] = prev_output_cmd.pos[i] + max_vel * dt * delta_pos / std::abs(delta_pos);
                _logger->debug("Joint {} pos {:.3f} pos cmd clipped: {:.3f} to {:.3f}", i, _joint_state.pos[i],
                               _output_joint_cmd.pos[i], _output_joint_cmd.pos[i]);
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
                _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos + _robot_config.gripper_vel_max * dt *
                                                                                  gripper_delta_pos /
                                                                                  std::abs(gripper_delta_pos);
                if (std::abs(_output_joint_cmd.gripper_pos - _output_joint_cmd.gripper_pos) >= 0.001)
                    _logger->debug("Gripper pos cmd clipped: {:.3f} to {:.3f}", _output_joint_cmd.gripper_pos,
                                   _output_joint_cmd.gripper_pos);
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
            _logger->debug("Gripper torque is too large, gripper pos cmd is not updated");
            _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos;
        }
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

double Arx5JointController::get_timestamp()
{
    return double(get_time_us() - _start_time_us) / 1e6;
}

void Arx5JointController::recv_once()
{
    int communicate_sleep_us = 150;
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
            _can_handle.enable_DM_motor(_robot_config.motor_id[i]);
        }
        else
        {
            _logger->error("Motor type not supported.");
            assert(false);
        }
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }
    if (_robot_config.gripper_motor_type == MotorType::DM_J4310)
    {
        int start_send_motor_time_us = get_time_us();
        _can_handle.enable_DM_motor(_robot_config.gripper_motor_id);
        int finish_send_motor_time_us = get_time_us();
        sleep_us(communicate_sleep_us - (finish_send_motor_time_us - start_send_motor_time_us));
    }
    sleep_ms(1); // Wait until all the messages are updated
    _update_joint_state();
}

void Arx5JointController::_update_joint_state()
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

void Arx5JointController::_send_recv()
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
            return;
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

void Arx5JointController::_check_joint_state_sanity()
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

void Arx5JointController::_over_current_protection()
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

void Arx5JointController::_enter_emergency_state()
{
    Gain damping_gain{_robot_config.joint_dof};
    damping_gain.kd = _controller_config.default_kd;
    damping_gain.kd[1] *= 3;
    damping_gain.kd[2] *= 3;
    damping_gain.kd[3] *= 1.5;
    set_gain(damping_gain);
    _input_joint_cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
    _input_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);

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

void Arx5JointController::_background_send_recv()
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

void Arx5JointController::set_joint_cmd(JointState new_cmd)
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    if (new_cmd.gripper_vel != 0 || new_cmd.gripper_torque != 0)
    {
        _logger->warn("Gripper vel and torque control is not supported yet.");
        new_cmd.gripper_vel = 0;
        new_cmd.gripper_torque = 0;
    }
    if (new_cmd.timestamp != 0 && new_cmd.timestamp < get_timestamp())
    {
        _logger->warn("Joint command timestamp ({:.4f}s) is not 0 but in the past (current timestamp: {:.4f}s). New "
                      "joint command "
                      "is ignored.",
                      new_cmd.timestamp, get_timestamp());
        return;
    }
    _input_joint_cmd = new_cmd;
    _interp_start_joint_cmd = _intermediate_joint_cmd;
    _interp_start_joint_cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
}

std::tuple<JointState, JointState> Arx5JointController::get_joint_cmd()
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    return std::make_tuple(_input_joint_cmd, _output_joint_cmd);
}

Gain Arx5JointController::get_gain()
{
    return _gain;
}

void Arx5JointController::set_gain(Gain new_gain)
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

void Arx5JointController::reset_to_home()
{
    JointState cmd{_robot_config.joint_dof};
    Gain gain{_robot_config.joint_dof};
    JointState init_state = get_state();
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
    _logger->debug("init_state.pos: {}, target_state.pos: {}", vec2str(init_state.pos), vec2str(target_state.pos));
    if (init_state.pos == VecDoF::Zero(_robot_config.joint_dof))
    {
        _logger->error("Motor positions are not initialized. Please check the connection.");
        _background_send_recv_running = false;
        throw std::runtime_error("Motor positions are not initialized. Please check the connection.");
    }
    // calculate the maximum joint position error
    double max_pos_error = (init_state.pos - VecDoF::Zero(_robot_config.joint_dof)).cwiseAbs().maxCoeff();
    max_pos_error = std::max(max_pos_error, init_state.gripper_pos * 2 / _robot_config.gripper_width);
    // interpolate from current kp kd to default kp kd in max(max_pos_error, 0.5)s
    // and keep the target for max(max_pos_error, 0.5)s
    double step_num = std::max(max_pos_error, 0.5) / _controller_config.controller_dt;
    _logger->info("Start reset to home in {:.3f}s, max_pos_error: {:.3f}", std::max(max_pos_error, double(0.5)) + 0.5,
                  max_pos_error);

    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = true;
    for (int i = 0; i <= step_num; ++i)
    {
        double alpha = double(i) / step_num;
        gain = init_gain * (1 - alpha) + target_gain * alpha;
        cmd = init_state * (1 - alpha) + target_state * alpha;
        cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
        cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
        cmd.timestamp = 0;
        set_joint_cmd(cmd);
        sleep_ms(5);
        set_gain(gain);
    }
    // Hardcode 0.5 s
    sleep_ms(500);
    _logger->info("Finish reset to home");
    _background_send_recv_running = prev_running;
}

void Arx5JointController::set_to_damping()
{
    JointState cmd{_robot_config.joint_dof};
    JointState state{_robot_config.joint_dof};
    Gain gain{_robot_config.joint_dof};
    JointState init_state = get_state();
    Gain init_gain = get_gain();
    Gain target_gain{_robot_config.joint_dof};
    target_gain.kd = _controller_config.default_kd;
    _logger->info("Start set to damping");
    //  interpolate from current kp kd to default kp kd in 0.5s
    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = true;
    int step_num = 20; // 0.1s in total
    for (int i = 0; i <= step_num; ++i)
    {
        state = get_state();
        cmd.pos = state.pos;
        cmd.gripper_pos = state.gripper_pos;
        cmd.torque = VecDoF::Zero(_robot_config.joint_dof);
        cmd.vel = VecDoF::Zero(_robot_config.joint_dof);
        double alpha = double(i) / double(step_num);
        gain = init_gain * (1.0 - alpha) + target_gain * alpha;
        set_gain(gain);
        set_joint_cmd(cmd);
        sleep_ms(5);
    }
    sleep_ms(500);
    _logger->info("Finish set to damping");
    _background_send_recv_running = prev_running;
}

void Arx5JointController::calibrate_gripper()
{
    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = false;
    sleep_us(1000);
    for (int i = 0; i < 10; ++i)
    {
        _can_handle.send_DM_motor_cmd(_robot_config.gripper_motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    _logger->info("Start calibrating gripper. Please fully close the gripper and press "
                  "enter to continue");
    std::cin.get();
    _can_handle.reset_zero_readout(_robot_config.gripper_motor_id);
    usleep(400);
    for (int i = 0; i < 10; ++i)
    {
        _can_handle.send_DM_motor_cmd(_robot_config.gripper_motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    usleep(400);
    _logger->info("Finish setting zero point. Please fully open the gripper and press "
                  "enter to continue");
    std::cin.get();

    for (int i = 0; i < 10; ++i)
    {
        _can_handle.send_DM_motor_cmd(_robot_config.gripper_motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
    std::cout << "Fully-open joint position readout: " << motor_msg[_robot_config.gripper_motor_id].angle_actual_rad
              << std::endl;
    std::cout << "  Please update the _robot_config.gripper_open_readout value in config.h to finish gripper "
                 "calibration."
              << std::endl;
    if (prev_running)
    {
        _background_send_recv_running = true;
    }
}

void Arx5JointController::calibrate_joint(int joint_id)
{
    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = false;
    sleep_us(1000);
    int motor_id = _robot_config.motor_id[joint_id];
    for (int i = 0; i < 10; ++i)
    {
        if (_robot_config.motor_type[joint_id] == MotorType::EC_A4310)
            _can_handle.send_EC_motor_cmd(motor_id, 0, 0, 0, 0, 0);
        else
            _can_handle.send_DM_motor_cmd(motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    _logger->info("Start calibrating joint {}. Please move the joint to the home position and press enter to continue",
                  joint_id);
    std::cin.get();
    if (_robot_config.motor_type[joint_id] == MotorType::EC_A4310)
        _can_handle.can_cmd_init(motor_id, 0x03);
    else
        _can_handle.reset_zero_readout(motor_id);
    usleep(400);
    for (int i = 0; i < 10; ++i)
    {
        if (_robot_config.motor_type[joint_id] == MotorType::EC_A4310)
            _can_handle.send_EC_motor_cmd(motor_id, 0, 0, 0, 0, 0);
        else
            _can_handle.send_DM_motor_cmd(motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    usleep(400);
    _logger->info("Finish setting zero point for joint {}.", joint_id);
    if (prev_running)
    {
        _background_send_recv_running = true;
    }
}

void Arx5JointController::set_log_level(spdlog::level::level_enum log_level)
{
    _logger->set_level(log_level);
}
