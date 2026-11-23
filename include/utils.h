#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"
#include <Eigen/Core>
#include <string>
#include <vector>
namespace arx
{
class MovingAverageXd
{
  public:
    MovingAverageXd(int dof, int window_size);
    ~MovingAverageXd();

    void set_window_size(int window_size);
    void reset();
    Eigen::VectorXd filter(Eigen::VectorXd new_data);

  private:
    int dof_;
    int window_size_;
    int window_index_;
    Eigen::VectorXd window_sum_;
    Eigen::MatrixXd window_;
};

class JointStateInterpolator
{
  public:
    JointStateInterpolator(int dof, std::string method);
    ~JointStateInterpolator() = default;
    void init(JointState start_state, JointState end_state);
    void init_fixed(JointState start_state);
    void append_waypoint(double current_time, JointState end_state);
    void append_traj(double current_time, std::vector<JointState> traj);
    void override_waypoint(double current_time, JointState end_state);
    void override_traj(double current_time, std::vector<JointState> traj);
    JointState interpolate(double time);
    std::string to_string();
    bool is_initialized();

  private:
    int dof_;
    bool initialized_ = false;
    std::string method_;
    std::vector<JointState> traj_;
};

void calc_joint_vel(std::vector<JointState> &traj, double avg_window_s = 0.05);
// std::string vec2str(const Eigen::VectorXd& vec, int precision = 3);

std::string joint_traj2str(const std::vector<JointState> &traj, int precision = 3);
std::string state2str(const JointState &state, int precision = 3);

std::string get_urdf(std::string model);

// Writes a copy of `urdf_path` in which only the <mass value> of the last link (the parent of `eef_link_name`) is
// replaced by `mass_kg`, and returns its path: a new file per call in std::filesystem::temp_directory_path()
// (follows TMPDIR), named arx5_<urdf stem>_<link>_XXXXXX.urdf, mode 0600. The caller keeps the file; it is never
// deleted. Throws std::invalid_argument if `mass_kg` is not finite, <= 0 or > 10 kg, and std::runtime_error if the
// URDF cannot be read, the link or its <inertial><mass> is missing, or the copy cannot be written.
std::string patch_urdf_last_link_mass(const std::string &urdf_path, const std::string &eef_link_name, double mass_kg);

std::string get_root_dir();

} // namespace arx
std::string vec2str(const Eigen::VectorXd &vec, int precision = 3);

#endif
