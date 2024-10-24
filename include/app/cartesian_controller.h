#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

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
class Arx5CartesianController : public Arx5ControllerBase
{
  public:
    Arx5CartesianController(RobotConfig robot_config, ControllerConfig controller_config, std::string interface_name,
                            std::string urdf_path);
    Arx5CartesianController(std::string model, std::string interface_name, std::string urdf_path);

    void set_eef_cmd(EEFState new_cmd);
    void set_eef_traj(std::vector<EEFState> new_traj);
    EEFState get_eef_cmd();
};
} // namespace arx

#endif // CARTESIAN_CONTROLLER