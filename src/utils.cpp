
#include "utils.h"
#include <cstdarg>
#include <cstdio>

namespace arx
{

MovingAverageXd::MovingAverageXd(int dof, int window_size)
{
    _dof = dof;
    _window_size = window_size;
    reset();
}

MovingAverageXd::~MovingAverageXd()
{
}

void MovingAverageXd::reset()
{
    _window_index = 0;
    _window_sum = Eigen::VectorXd::Zero(_dof);
    _window = Eigen::MatrixXd::Zero(_window_size, _dof);
}

Eigen::VectorXd MovingAverageXd::filter(Eigen::VectorXd new_data)
{
    _window_sum -= _window.row(_window_index);
    _window_sum += new_data;
    _window.row(_window_index) = new_data;
    _window_index = (_window_index + 1) % _window_size;
    return _window_sum / _window_size;
}

// std::string vec2str(const Eigen::VectorXd& vec, int precision) {
//   std::string str = "[";
//   for (int i = 0; i < vec.size(); i++) {
//     char buffer[50];
//     std::sprintf(buffer, "%.*f", precision, vec(i));
//     str += buffer;
//     if (i < vec.size() - 1) {
//       str += ", ";
//     }
//   }
//   str += "]";
//   return str;
// }

Interpolator1d::Interpolator1d(std::string method)
{
    if (method != "linear" && method != "cubic")
    {
        throw std::invalid_argument("Invalid interpolation method: " + method +
                                    ". Currently available: 'linear' or 'cubic'");
    }
    _method = method;
    _initialized = false;
}

void Interpolator1d::init(double start_pos, double start_vel, double start_time, double end_pos, double end_vel,
                          double end_time)
{
    if (end_time < start_time)
    {
        throw std::invalid_argument("End time must be no less than start time");
    }
    else if (end_time == start_time && start_pos != end_pos)
    {
        throw std::invalid_argument("Start and end time are the same, but start and end positions are different");
    }

    _start_pos = start_pos;
    _start_vel = start_vel;
    _start_time = start_time;
    _end_pos = end_pos;
    _end_vel = end_vel;
    _end_time = end_time;
    _fixed = false;
    _initialized = true;
}

void Interpolator1d::init_fixed(double start_pos)
{
    _start_pos = start_pos;
    _start_vel = 0;
    _start_time = 0;
    _end_pos = start_pos;
    _end_vel = 0;
    _end_time = 0;

    _fixed = true;
    _initialized = true;
}

void Interpolator1d::update(double current_time, double end_pos, double end_vel, double end_time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    double current_pos = 0;
    double current_vel = 0;

    if (current_time > _end_time)
    {
        current_vel = 0;
        current_pos = _end_pos;
    }
    else if (current_time < _start_time)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_pos = interpolate_pos(current_time);
        current_vel = interpolate_vel(current_time);
    }

    if (end_time < current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }
    else if (end_time == current_time && end_pos != _end_pos)
    {
        throw std::invalid_argument("Current and end time are the same, but current and end positions are different");
    }

    _start_pos = current_pos;
    _start_vel = current_vel;
    _start_time = current_time;
    _end_pos = end_pos;
    _end_vel = end_vel;
    _end_time = end_time;
    _fixed = false;
}

double Interpolator1d::interpolate_pos(double time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (_fixed)
    {
        return _start_pos;
    }

    if (time <= _start_time)
    {
        return _start_pos;
    }
    else if (time >= _end_time)
    {
        return _end_pos;
    }
    if (_method == "linear")
    {
        return _start_pos + (_end_pos - _start_pos) * (time - _start_time) / (_end_time - _start_time);
    }
    else if (_method == "cubic")
    {
        double pos = 0;

        double t = (time - _start_time) / (_end_time - _start_time);
        double t2 = t * t;
        double t3 = t2 * t;
        double a = 2 * t3 - 3 * t2 + 1;
        double b = t3 - 2 * t2 + t;
        double c = -2 * t3 + 3 * t2;
        double d = t3 - t2;

        pos = a * _start_pos + b * _start_vel + c * _end_pos + d * _end_vel;

        return pos;
    }
}

double Interpolator1d::interpolate_vel(double time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (_fixed)
    {
        return 0.0;
    }

    if (time < _start_time || time > _end_time)
    {
        throw std::invalid_argument("Time must be within the range of start and end time");
    }
    else if (time == _start_time)
    {
        return _start_vel;
    }
    else if (time == _end_time)
    {
        return _end_vel;
    }

    if (_method == "linear")
    {
        return (_end_pos - _start_pos) / (_end_time - _start_time);
    }
    else if (_method == "cubic")
    {
        double vel = 0;

        double t = (time - _start_time) / (_end_time - _start_time);
        double t2 = t * t;
        double a = 6 * t2 - 6 * t;
        double b = 3 * t2 - 4 * t + 1;
        double c = -6 * t2 + 6 * t;
        double d = 3 * t2 - 2 * t;
        vel = a * _start_pos + b * _start_vel + c * _end_pos + d * _end_vel;

        return vel;
    }
}

InterpolatorXd::InterpolatorXd(int dof, std::string method)
{
    if (method != "linear" && method != "cubic")
    {
        throw std::invalid_argument("Invalid interpolation method: " + method +
                                    ". Currently available: 'linear' or 'cubic'");
    }
    _dof = dof;
    _method = method;
    _initialized = false;
}

void InterpolatorXd::init(Eigen::VectorXd start_pos, Eigen::VectorXd start_vel, double start_time,
                          Eigen::VectorXd end_pos, Eigen::VectorXd end_vel, double end_time)
{
    if (end_time < start_time)
    {
        throw std::invalid_argument("End time must be no less than start time");
    }
    else if (end_time == start_time)
    {
        throw std::invalid_argument("Start and end time are the same, plsease use init_fixed() instead");
    }

    _start_pos = start_pos;
    _start_vel = start_vel;
    _start_time = start_time;
    _end_pos = end_pos;
    _end_vel = end_vel;
    _end_time = end_time;
    _fixed = false;
    _initialized = true;
}

void InterpolatorXd::init_fixed(Eigen::VectorXd start_pos)
{
    _start_pos = start_pos;
    _start_vel = Eigen::VectorXd::Zero(_dof);
    _start_time = 0;
    _end_pos = start_pos;
    _end_vel = Eigen::VectorXd::Zero(_dof);
    _end_time = 0;

    _fixed = true;
    _initialized = true;
}

void InterpolatorXd::update(double current_time, Eigen::VectorXd end_pos, Eigen::VectorXd end_vel, double end_time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    Eigen::VectorXd current_pos(_dof);
    Eigen::VectorXd current_vel(_dof);

    if (current_time > _end_time)
    {
        current_vel = Eigen::VectorXd::Zero(_dof);
        current_pos = _end_pos;
    }
    else if (current_time < _start_time)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_pos = interpolate_pos(current_time);
        current_vel = interpolate_vel(current_time);
    }

    if (end_time < current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }
    else if (end_time == current_time && end_pos != _end_pos)
    {
        throw std::invalid_argument("Current and end time are the same, but current and end positions are different");
    }

    _start_pos = current_pos;
    _start_vel = current_vel;
    _start_time = current_time;
    _end_pos = end_pos;
    _end_vel = end_vel;
    _end_time = end_time;
    _fixed = false;
}

Eigen::VectorXd InterpolatorXd::interpolate_pos(double time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (_fixed)
    {
        return _start_pos;
    }

    if (time <= _start_time)
    {
        return _start_pos;
    }
    else if (time >= _end_time)
    {
        return _end_pos;
    }
    if (_method == "linear")
    {
        return _start_pos + (_end_pos - _start_pos) * (time - _start_time) / (_end_time - _start_time);
    }
    else if (_method == "cubic")
    {
        Eigen::VectorXd pos = Eigen::VectorXd::Zero(_dof);
        for (int i = 0; i < _dof; i++)
        {
            double t = (time - _start_time) / (_end_time - _start_time);
            double t2 = t * t;
            double t3 = t2 * t;
            double a = 2 * t3 - 3 * t2 + 1;
            double b = t3 - 2 * t2 + t;
            double c = -2 * t3 + 3 * t2;
            double d = t3 - t2;
            pos(i) = a * _start_pos(i) + b * _start_vel(i) + c * _end_pos(i) + d * _end_vel(i);
        }
        return pos;
    }
}

Eigen::VectorXd InterpolatorXd::interpolate_vel(double time)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (_fixed)
    {
        return Eigen::VectorXd::Zero(_dof);
    }

    if (time < _start_time || time > _end_time)
    {
        throw std::invalid_argument("Time must be within the range of start and end time");
    }
    else if (time == _start_time)
    {
        return _start_vel;
    }
    else if (time == _end_time)
    {
        return _end_vel;
    }

    if (_method == "linear")
    {
        return (_end_pos - _start_pos) / (_end_time - _start_time);
    }
    else if (_method == "cubic")
    {
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(_dof);
        for (int i = 0; i < _dof; i++)
        {
            double t = (time - _start_time) / (_end_time - _start_time);
            double t2 = t * t;
            double a = 6 * t2 - 6 * t;
            double b = 3 * t2 - 4 * t + 1;
            double c = -6 * t2 + 6 * t;
            double d = 3 * t2 - 2 * t;
            vel(i) = a * _start_pos(i) + b * _start_vel(i) + c * _end_pos(i) + d * _end_vel(i);
        }
        return vel;
    }
}
} // namespace arx

std::string vec2str(const Eigen::VectorXd &vec, int precision)
{
    std::string str = "[";
    for (int i = 0; i < vec.size(); i++)
    {
        char buffer[50];
        std::sprintf(buffer, "%.*f", precision, vec(i));
        str += buffer;
        if (i < vec.size() - 1)
        {
            str += ", ";
        }
    }
    str += "]";
    return str;
}