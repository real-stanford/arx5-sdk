
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

JointStateInterpolator::JointStateInterpolator(int dof, std::string method) : _start_state{dof}, _end_state{dof}
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

void JointStateInterpolator::init(JointState start_state, JointState end_state)
{
    if (end_state.timestamp < start_state.timestamp)
    {
        throw std::invalid_argument("End time must be no less than start time");
    }
    else if (end_state.timestamp == start_state.timestamp)
    {
        throw std::invalid_argument("Start and end time are the same, plsease use init_fixed() instead");
    }
    _start_state = start_state;
    _end_state = end_state;
    _fixed = false;
    _initialized = true;
}

void JointStateInterpolator::init_fixed(JointState start_state)
{
    _start_state = start_state;
    _end_state = start_state;

    _fixed = true;
    _initialized = true;
}

void JointStateInterpolator::update(double current_time, JointState end_state)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (end_state.timestamp < current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }
    else if (end_state.timestamp == current_time && end_state.pos != _end_state.pos)
    {
        throw std::invalid_argument("Current and end time are the same, but current and end positions are different");
    }

    JointState current_state{_dof};

    if (current_time < _start_state.timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }
    _start_state = current_state;
    _end_state = end_state;
    _fixed = false;
}

JointState JointStateInterpolator::interpolate(double time)
{

    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }
    if (time <= 0)
    {
        throw std::invalid_argument("Interpolate time must be greater than 0");
    }

    if (_fixed)
    {
        return _start_state;
    }

    if (time <= _start_state.timestamp)
    {
        return _start_state;
    }
    else if (time >= _end_state.timestamp)
    {
        return _end_state;
    }
    if (_method == "linear")
    {
        JointState interp_result = _start_state + (_end_state - _start_state) * (time - _start_state.timestamp) /
                                                      (_end_state.timestamp - _start_state.timestamp);
        interp_result.timestamp = time;
        return interp_result;
    }
    else if (_method == "cubic")
    {
        // Torque and gripper pos will still be linearly interpolated
        JointState interp_result = _start_state + (_end_state - _start_state) * (time - _start_state.timestamp) /
                                                      (_end_state.timestamp - _start_state.timestamp);
        interp_result.timestamp = time;

        // Cubic interpolation for pos and vel
        double t = (time - _start_state.timestamp) / (_end_state.timestamp - _start_state.timestamp);
        double t2 = t * t;
        double t3 = t2 * t;
        double pos_a = 2 * t3 - 3 * t2 + 1;
        double pos_b = t3 - 2 * t2 + t;
        double pos_c = -2 * t3 + 3 * t2;
        double pos_d = t3 - t2;
        interp_result.pos =
            pos_a * _start_state.pos + pos_b * _start_state.vel + pos_c * _end_state.pos + pos_d * _end_state.vel;

        double vel_a = 6 * t2 - 6 * t;
        double vel_b = 3 * t2 - 4 * t + 1;
        double vel_c = -6 * t2 + 6 * t;
        double vel_d = 3 * t2 - 2 * t;
        interp_result.vel =
            vel_a * _start_state.pos + vel_b * _start_state.vel + vel_c * _end_state.pos + vel_d * _end_state.vel;
        return interp_result;
    }
}

std::string JointStateInterpolator::to_string()
{
    std::string str = "JointStateInterpolator: \n";
    str += "  start: " + state2str(_start_state);
    str += "  end: " + state2str(_end_state);
    return str;
}

std::string state2str(const JointState &state, int precision)
{
    std::string str = "";
    str += "pos:" + vec2str(state.pos, precision);
    str += " vel:" + vec2str(state.vel, precision);
    str += " torque:" + vec2str(state.torque, precision);
    str += " gripper_pos:" + std::to_string(state.gripper_pos);
    str += "\n";
    return str;
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
