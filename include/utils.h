#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"
#include <Eigen/Core>

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

class Interpolator1d
{
  public:
    Interpolator1d(std::string method);
    ~Interpolator1d() = default;
    void init(double start_pos, double start_vel, double start_time, double end_pos, double end_vel, double end_time);
    void init_fixed(double start_pos);
    void update(double current_time, double end_pos, double end_vel, double end_time);
    double interpolate_pos(double time);
    double interpolate_vel(double time);

  private:
    bool _initialized = false;
    bool _fixed = false;
    std::string _method;
    double _start_pos;
    double _start_vel;
    double _start_time;
    double _end_pos;
    double _end_vel;
    double _end_time;
};

class InterpolatorXd
{
  public:
    InterpolatorXd(int dof, std::string method);
    ~InterpolatorXd() = default;
    void init(Eigen::VectorXd start_pos, Eigen::VectorXd start_vel, double start_time, Eigen::VectorXd end_pos,
              Eigen::VectorXd end_vel, double end_time);
    void init_fixed(Eigen::VectorXd start_pos);
    void update(double current_time, Eigen::VectorXd end_pos, Eigen::VectorXd end_vel, double end_time);
    Eigen::VectorXd interpolate_pos(double time);
    Eigen::VectorXd interpolate_vel(double time);

  private:
    int _dof;
    bool _initialized = false;
    bool _fixed = false;
    std::string _method;
    Eigen::VectorXd _start_pos;
    Eigen::VectorXd _start_vel;
    double _start_time;
    Eigen::VectorXd _end_pos;
    Eigen::VectorXd _end_vel;
    double _end_time;
};

// std::string vec2str(const Eigen::VectorXd& vec, int precision = 3);

} // namespace arx
std::string vec2str(const Eigen::VectorXd &vec, int precision = 3);

#endif
