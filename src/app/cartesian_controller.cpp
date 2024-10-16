#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/config.h"
#include "utils.h"
#include <stdexcept>
#include <sys/syscall.h>
#include <sys/types.h>

using namespace arx;

Arx5CartesianController::Arx5CartesianController(RobotConfig robot_config, ControllerConfig controller_config,
                                                 std::string interface_name, std::string urdf_path)
    : Arx5ControllerBase(robot_config, controller_config, interface_name, urdf_path)
{
}

Arx5CartesianController::Arx5CartesianController(std::string model, std::string interface_name, std::string urdf_path)
    : Arx5CartesianController::Arx5CartesianController(
          RobotConfigFactory::get_instance().get_config(model),
          ControllerConfigFactory::get_instance().get_config(
              "cartesian_controller", RobotConfigFactory::get_instance().get_config(model).joint_dof),
          interface_name, urdf_path)
{
}

void Arx5CartesianController::set_eef_cmd(EEFState new_cmd)
{
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    JointState current_joint_state = get_joint_state();

    // The following line only works under c++17
    // auto [success, target_joint_pos] = _solver->inverse_kinematics(new_cmd.pose_6d, current_joint_state.pos);

    std::tuple<bool, VecDoF> ik_results;
    ik_results = _solver->multi_trial_ik(new_cmd.pose_6d, _joint_state.pos);
    bool success = std::get<0>(ik_results);

    if (new_cmd.timestamp == 0)
        new_cmd.timestamp = get_timestamp() + _controller_config.default_preview_time;

    JointState target_joint_state{_robot_config.joint_dof};
    target_joint_state.pos = std::get<1>(ik_results);
    target_joint_state.gripper_pos = new_cmd.gripper_pos;
    target_joint_state.timestamp = new_cmd.timestamp;

    if (success)
    {
        double current_time = get_timestamp();
        // TODO: include velocity
        std::lock_guard<std::mutex> lock(_cmd_mutex);
        _interpolator.update(current_time, target_joint_state);
    }
    else
    {
        _logger->warn("Inverse kinematics failed");
    }
}
