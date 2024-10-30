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
    ik_results = multi_trial_ik(new_cmd.pose_6d, _joint_state.pos);
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
        ik_results = multi_trial_ik(eef_state.pose_6d, current_joint_state.pos);
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

std::tuple<int, Eigen::VectorXd> Arx5CartesianController::multi_trial_ik(Eigen::Matrix<double, 6, 1> target_pose_6d,
                                                                         Eigen::VectorXd current_joint_pos,
                                                                         int additional_trial_num)
{
    if (additional_trial_num < 0)
        throw std::invalid_argument("Number of additional trials must be non-negative");
    // Solve IK with at least 2 init joint positions: current joint position and home joint position
    if (current_joint_pos.size() != _robot_config.joint_dof || target_pose_6d.size() != 6)
        throw std::invalid_argument(
            "Inverse kinematics input expected size 6, " + std::to_string(_robot_config.joint_dof) + " but got " +
            std::to_string(target_pose_6d.size()) + ", " + std::to_string(current_joint_pos.size()));
    Eigen::MatrixXd init_joint_positions = Eigen::MatrixXd::Zero(additional_trial_num + 2, _robot_config.joint_dof);
    init_joint_positions.row(0) = current_joint_pos;
    init_joint_positions.row(1) = Eigen::VectorXd::Zero(_robot_config.joint_dof);
    for (int i = 0; i < additional_trial_num; i++)
    {
        init_joint_positions.row(i + 2) = Eigen::VectorXd::Random(_robot_config.joint_dof);
        // Map the random values into the joint limits
        for (int j = 0; j < _robot_config.joint_dof; j++)
        {
            init_joint_positions(i + 2, j) =
                _robot_config.joint_pos_min[j] + (init_joint_positions(i + 2, j) + 1) / 2 *
                                                     (_robot_config.joint_pos_max[j] - _robot_config.joint_pos_min[j]);
        }
    }
    Eigen::MatrixXd target_joint_positions = Eigen::MatrixXd::Zero(additional_trial_num + 2, _robot_config.joint_dof);
    std::vector<int> all_ik_status(additional_trial_num + 2, 0);
    std::vector<double> distances(additional_trial_num + 2, 100000); // L2 distances, initialize to infinity

    for (int i = 0; i < additional_trial_num + 2; i++)
    {
        std::tuple<int, Eigen::VectorXd> result;
        result = _solver->inverse_kinematics(target_pose_6d, init_joint_positions.row(i));
        int ik_status = std::get<0>(result);
        Eigen::VectorXd target_joint_pos = std::get<1>(result);
        bool in_joint_limit = ((_robot_config.joint_pos_max - target_joint_pos).array() > 0).all() &&
                              ((_robot_config.joint_pos_min - target_joint_pos).array() < 0).all();
        if (ik_status == 0 && !in_joint_limit)
        {
            ik_status = -9; // E_EXCEED_JOINT_LIMIT
        }
        all_ik_status[i] = ik_status;
        // Check whether the target joint position is within the joint limits
        distances[i] = (target_joint_pos - current_joint_pos).norm();
        target_joint_positions.row(i) = target_joint_pos;
    }
    bool final_success = std::any_of(all_ik_status.begin(), all_ik_status.end(), [](bool success) { return success; });
    int min_idx = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
    int min_ik_status = all_ik_status[min_idx];
    Eigen::VectorXd min_target_joint_pos = target_joint_positions.row(min_idx);
    // clip the target joint position to the joint limits
    for (int i = 0; i < _robot_config.joint_dof; i++)
    {
        min_target_joint_pos[i] =
            std::max(_robot_config.joint_pos_min[i], std::min(_robot_config.joint_pos_max[i], min_target_joint_pos[i]));
    }
    return std::make_tuple(min_ik_status, min_target_joint_pos);
}
