#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H
#include "app/common.h"
#include "app/config.h"
#include "app/controller_base.h"
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

class Arx5JointController : public Arx5ControllerBase
{
  public:
    Arx5JointController(RobotConfig robot_config, ControllerConfig controller_config, std::string interface_name,
                        std::string urdf_path);
    Arx5JointController(std::string model, std::string interface_name, std::string urdf_path);

    void set_joint_cmd(JointState new_cmd);

    // Only works when background_send_recv is disabled
    void send_recv_once();
    void recv_once();

    void calibrate_gripper();
    void calibrate_joint(int joint_id);
};

} // namespace arx

#endif