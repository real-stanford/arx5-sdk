#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"
#include <Eigen/Core>
#include <string>

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
    int _dof;
    int _window_size;
    int _window_index;
    Eigen::VectorXd _window_sum;
    Eigen::MatrixXd _window;
};

class JointStateInterpolator
{
  public:
    JointStateInterpolator(int dof, std::string method);
    ~JointStateInterpolator() = default;
    void init(JointState start_state, JointState end_state);
    void init_fixed(JointState start_state);
    void update(double current_time, JointState end_state);
    JointState interpolate(double time);
    std::string to_string();

  private:
    int _dof;
    bool _initialized = false;
    bool _fixed = false;
    std::string _method;
    JointState _start_state;
    JointState _end_state;
};

// std::string vec2str(const Eigen::VectorXd& vec, int precision = 3);

std::string state2str(const JointState &state, int precision = 3);

} // namespace arx
std::string vec2str(const Eigen::VectorXd &vec, int precision = 3);

#endif
