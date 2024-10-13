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
    std::tuple<bool, VecDoF> ik_results;
    ik_results = _solver->inverse_kinematics(new_cmd.pose_6d, _joint_state.pos);
    bool success = std::get<0>(ik_results);
    VecDoF target_joint_pos = std::get<1>(ik_results);
    if (success)
    {
        double current_time = get_timestamp();
        // TODO: include velocity

        std::lock_guard<std::mutex> lock(_interpolator_mutex);
        _joint_interpolator.update(current_time, target_joint_pos, Pose6d::Zero(), new_cmd.timestamp);
    }
    else
    {
        _logger->warn("Inverse kinematics failed");
    }
}

Pose6d Arx5CartesianController::get_home_pose()
{
    return _solver->forward_kinematics(VecDoF::Zero(_robot_config.joint_dof));
}
