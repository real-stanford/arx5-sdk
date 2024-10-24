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
    if (!controller_config.background_send_recv)
        throw std::runtime_error(
            "controller_config.background_send_recv should be set to true when running cartesian controller.");
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
    JointState current_joint_state = get_joint_state();

    // The following line only works under c++17
    // auto [success, target_joint_pos] = _solver->inverse_kinematics(new_cmd.pose_6d, current_joint_state.pos);

    std::tuple<int, VecDoF> ik_results;
    ik_results = _solver->multi_trial_ik(new_cmd.pose_6d, _joint_state.pos);
    int ik_status = std::get<0>(ik_results);

    if (new_cmd.timestamp == 0)
        new_cmd.timestamp = get_timestamp() + _controller_config.default_preview_time;

    JointState target_joint_state{_robot_config.joint_dof};
    target_joint_state.pos = std::get<1>(ik_results);
    target_joint_state.gripper_pos = new_cmd.gripper_pos;
    target_joint_state.timestamp = new_cmd.timestamp;

    double current_time = get_timestamp();
    // TODO: include velocity
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    _interpolator.override_waypoint(get_timestamp(), target_joint_state);

    if (ik_status != 0)
    {
        _logger->warn("Inverse kinematics failed: {} ({})", _solver->get_ik_status_name(ik_status), ik_status);
    }
}

void Arx5CartesianController::set_eef_traj(std::vector<EEFState> new_traj)
{
    std::vector<JointState> joint_traj;
    for (auto eef_state : new_traj)
    {
        if (eef_state.timestamp == 0)
            throw std::invalid_argument("EEFState timestamp must be set for all waypoints");
        JointState current_joint_state = get_joint_state();
        std::tuple<int, VecDoF> ik_results;
        ik_results = _solver->multi_trial_ik(eef_state.pose_6d, current_joint_state.pos);
        int ik_status = std::get<0>(ik_results);

        JointState target_joint_state{_robot_config.joint_dof};
        target_joint_state.pos = std::get<1>(ik_results);
        target_joint_state.gripper_pos = eef_state.gripper_pos;
        target_joint_state.timestamp = eef_state.timestamp;

        joint_traj.push_back(target_joint_state);

        if (ik_status != 0)
        {
            _logger->warn("Inverse kinematics failed: {} ({})", _solver->get_ik_status_name(ik_status), ik_status);
        }
    }

    // Include velocity: first and last point based on current state, others based on neighboring points
    calc_joint_vel(joint_traj);

    double current_time = get_timestamp();
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    _interpolator.override_traj(get_timestamp(), joint_traj);
}

EEFState Arx5CartesianController::get_eef_cmd()
{
    JointState joint_cmd = get_joint_cmd();
    EEFState eef_cmd;
    eef_cmd.pose_6d = _solver->forward_kinematics(joint_cmd.pos);
    eef_cmd.gripper_pos = joint_cmd.gripper_pos;
    eef_cmd.timestamp = joint_cmd.timestamp;
    return eef_cmd;
}
