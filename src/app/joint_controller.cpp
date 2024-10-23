#include "app/joint_controller.h"
#include "utils.h"
#include <array>
#include <stdexcept>
#include <sys/syscall.h>
#include <sys/types.h>
using namespace arx;

Arx5JointController::Arx5JointController(RobotConfig robot_config, ControllerConfig controller_config,
                                         std::string interface_name, std::string urdf_path)
    : Arx5ControllerBase(robot_config, controller_config, interface_name, urdf_path)
{
}

Arx5JointController::Arx5JointController(std::string model, std::string interface_name, std::string urdf_path)
    : Arx5JointController::Arx5JointController(
          RobotConfigFactory::get_instance().get_config(model),
          ControllerConfigFactory::get_instance().get_config(
              "joint_controller", RobotConfigFactory::get_instance().get_config(model).joint_dof),
          interface_name, urdf_path)
{
}

void Arx5JointController::set_joint_cmd(JointState new_cmd)
{
    JointState current_joint_state = get_joint_state();
    double current_time = get_timestamp();
    if (new_cmd.timestamp == 0)
        new_cmd.timestamp = current_time + _controller_config.default_preview_time;

    std::lock_guard<std::mutex> lock(_cmd_mutex);
    if (abs(new_cmd.timestamp - current_time) < 1e-3)
        // If the new timestamp is close enough (<1ms) to the current time
        // Will override the entire interpolator object
        _interpolator.init_fixed(new_cmd);
    else
        _interpolator.update_traj(get_timestamp(), std::vector<JointState>{new_cmd});
}

void Arx5JointController::recv_once()
{
    if (_background_send_recv_running)
    {
        _logger->warn("send_recv task is already running in background. recv_once() is ignored.");
        return;
    }
    _recv();
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
