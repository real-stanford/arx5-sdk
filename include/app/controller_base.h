#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H
#include "app/common.h"
#include "app/config.h"
#include "app/solver.h"
#include "hardware/arx_can.h"
#include "utils.h"
#include <chrono>
#include <memory>
#include <mutex>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

namespace arx
{
class Arx5ControllerBase // parent class for the other two controllers
{
    std::tuple<JointState, JointState> get_joint_cmd();
    JointState get_joint_state();
    EEFState get_eef_state();
    void set_gain(Gain new_gain);
    Gain get_gain();

    double get_timestamp();
    RobotConfig get_robot_config();
    ControllerConfig get_controller_config();
    void set_log_level(spdlog::level::level_enum level);

  private:
    RobotConfig _robot_config;
    ControllerConfig _controller_config;

    int _over_current_cnt = 0;
    JointState _output_joint_cmd{_robot_config.joint_dof};
    JointState _intermediate_joint_cmd{_robot_config.joint_dof}; // _output_joint_cmd without gravity compensation
    JointState _input_joint_cmd{_robot_config.joint_dof};

    JointState _joint_state{_robot_config.joint_dof};
    Gain _gain{_robot_config.joint_dof};

    ArxCan _can_handle;
    std::shared_ptr<spdlog::logger> _logger;
    std::thread _background_send_recv_thread;

    bool _background_send_recv_running = false;
    bool _destroy_background_threads = false;

    std::mutex _cmd_mutex;
    std::mutex _state_mutex;
    std::mutex _interpolator_mutex;

    int _start_time_us;
    std::shared_ptr<Arx5Solver> _solver;

    void _init_robot();
    void _update_joint_state();
    void _update_output_cmd();
    void _send_recv();
    void _recv();
    void _check_joint_state_sanity();
    void _over_current_protection();
    void _background_send_recv();
    void _enter_emergency_state();
}
} // namespace arx