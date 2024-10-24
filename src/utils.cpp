
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

JointStateInterpolator::JointStateInterpolator(int dof, std::string method)
{
    if (method != "linear" && method != "cubic")
    {
        throw std::invalid_argument("Invalid interpolation method: " + method +
                                    ". Currently available: 'linear' or 'cubic'");
    }
    _dof = dof;
    _method = method;
    _initialized = false;
    _traj = std::vector<JointState>();
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
    if (start_state.pos.size() != _dof || end_state.pos.size() != _dof)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }
    _traj.clear();
    _traj.push_back(start_state);
    _traj.push_back(end_state);
    _initialized = true;
}

void JointStateInterpolator::init_fixed(JointState start_state)
{
    if (start_state.pos.size() != _dof)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }
    _traj.clear();
    _traj.push_back(start_state);
    _initialized = true;
}

void JointStateInterpolator::append_waypoint(double current_time, JointState end_state)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (end_state.pos.size() != _dof)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }

    if (end_state.timestamp <= current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }

    JointState current_state{_dof};

    if (current_time < _traj[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = _traj;
    _traj.clear();
    _traj.push_back(current_state);
    for (int i = 0; i < prev_traj.size(); i++)
    {
        if (prev_traj[i].timestamp > current_time)
        {
            if (prev_traj[i].timestamp > end_state.timestamp)
            {
                _traj.push_back(end_state);
                break;
            }
            else
            {
                _traj.push_back(prev_traj[i]);
            }
        }
        if (i == prev_traj.size() - 1)
        {
            _traj.push_back(end_state);
        }
    }
}

void JointStateInterpolator::override_waypoint(double current_time, JointState end_state)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (end_state.pos.size() != _dof)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }

    if (end_state.timestamp <= current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }

    JointState current_state{_dof};

    if (current_time < _traj[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = _traj;
    _traj.clear();
    _traj.push_back(current_state);
    _traj.push_back(end_state);
}

void JointStateInterpolator::append_traj(double current_time, std::vector<JointState> traj)
{
    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    // remove all the new traj points that are before current time
    while (traj.size() > 0 && traj[0].timestamp < current_time)
    {
        traj.erase(traj.begin());
    }

    if (traj.size() == 0)
    {
        printf("JointStateInterpolator::append_traj: Empty trajectory\n");
        return;
    }

    for (int i = 0; i < traj.size() - 1; i++)
    {
        if (traj[i].timestamp > traj[i + 1].timestamp)
        {
            throw std::invalid_argument("Trajectory timestamps must be in strictly ascending order");
        }
        if (traj[i].pos.size() != _dof || traj[i + 1].pos.size() != _dof)
        {
            throw std::invalid_argument("Joint state dimension mismatch");
        }
    }

    JointState current_state{_dof};
    if (current_time < _traj[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = _traj;
    _traj.clear();
    _traj.push_back(current_state);
    double new_traj_start_time = traj[0].timestamp;
    // Merge prev_traj before new_traj

    while (prev_traj.size() > 0 && prev_traj[0].timestamp < new_traj_start_time)
    {
        if (prev_traj[0].timestamp > current_time)
        {
            _traj.push_back(prev_traj[0]);
        }
        prev_traj.erase(prev_traj.begin());
    }
    while (traj.size() > 0)
    {
        _traj.push_back(traj[0]);
        traj.erase(traj.begin());
    }
}

void JointStateInterpolator::override_traj(double current_time, std::vector<JointState> traj)
{

    if (!_initialized)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    // remove all the new traj points that are before current time
    while (traj.size() > 0 && traj[0].timestamp < current_time)
    {
        traj.erase(traj.begin());
    }

    if (traj.size() == 0)
    {
        printf("JointStateInterpolator::append_traj: Empty trajectory\n");
        return;
    }

    for (int i = 0; i < traj.size() - 1; i++)
    {
        if (traj[i].timestamp > traj[i + 1].timestamp)
        {
            throw std::invalid_argument("Trajectory timestamps must be in strictly ascending order");
        }
        if (traj[i].pos.size() != _dof || traj[i + 1].pos.size() != _dof)
        {
            throw std::invalid_argument("Joint state dimension mismatch");
        }
    }

    JointState current_state{_dof};
    if (current_time < _traj[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = _traj;
    _traj.clear();
    _traj.push_back(current_state);

    while (traj.size() > 0)
    {
        _traj.push_back(traj[0]);
        traj.erase(traj.begin());
    }
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

    if (_traj.size() == 0)
    {
        throw std::runtime_error("Empty trajectory");
    }
    if (_traj.size() == 1)
    {
        JointState interp_state = _traj[0];
        interp_state.timestamp = time;
        return interp_state;
    }

    if (time <= _traj[0].timestamp)
    {
        JointState interp_state = _traj[0];
        interp_state.timestamp = time;
        return interp_state;
    }
    else if (time >= _traj.back().timestamp)
    {
        JointState interp_state = _traj.back();
        interp_state.timestamp = time;
        return interp_state;
    }

    for (int i = 0; i <= _traj.size() - 2; i++)
    {
        JointState start_state = _traj[i];
        JointState end_state = _traj[i + 1];
        if (time >= start_state.timestamp && time <= end_state.timestamp)
        {
            if (_method == "linear")
            {
                JointState interp_result = start_state + (end_state - start_state) * (time - start_state.timestamp) /
                                                             (end_state.timestamp - start_state.timestamp);
                interp_result.timestamp = time;
                return interp_result;
            }
            else if (_method == "cubic")
            {
                // Torque and gripper pos will still be linearly interpolated
                JointState interp_result = start_state + (end_state - start_state) * (time - start_state.timestamp) /
                                                             (end_state.timestamp - start_state.timestamp);
                interp_result.timestamp = time;

                // Cubic interpolation for pos and vel
                double t = (time - start_state.timestamp) / (end_state.timestamp - start_state.timestamp);
                double t2 = t * t;
                double t3 = t2 * t;
                double pos_a = 2 * t3 - 3 * t2 + 1;
                double pos_b = t3 - 2 * t2 + t;
                double pos_c = -2 * t3 + 3 * t2;
                double pos_d = t3 - t2;
                interp_result.pos =
                    pos_a * start_state.pos + pos_b * start_state.vel + pos_c * end_state.pos + pos_d * end_state.vel;

                double vel_a = 6 * t2 - 6 * t;
                double vel_b = 3 * t2 - 4 * t + 1;
                double vel_c = -6 * t2 + 6 * t;
                double vel_d = 3 * t2 - 2 * t;
                interp_result.vel =
                    vel_a * start_state.pos + vel_b * start_state.vel + vel_c * end_state.pos + vel_d * end_state.vel;
                return interp_result;
            }
        }
        if (i == _traj.size() - 2)
        {
            throw std::runtime_error("Interpolation failed");
        }
    }
}

std::string JointStateInterpolator::to_string()
{
    std::string str = "JointStateInterpolator DOF: " + std::to_string(_dof) + " Method: " + _method +
                      " Length: " + std::to_string(_traj.size()) + "\n";
    for (int i = 0; i < _traj.size(); i++)
    {
        str += state2str(_traj[i]);
    }

    return str;
}

void calc_joint_vel(std::vector<JointState> &traj)
{
    if (traj.size() < 2)
    {
        return;
    }
    for (int i = 0; i < traj.size(); i++)
    {
        int idx_0 = std::max(0, i - 2);
        int idx_1 = std::max(0, i - 1);
        int idx_2 = std::min((int)traj.size() - 1, i + 1);
        int idx_3 = std::min((int)traj.size() - 1, i + 2);
        traj[i].vel = (traj[idx_3].pos - traj[idx_0].pos) / (traj[idx_3].timestamp - traj[idx_0].timestamp) / 2 +
                      (traj[idx_2].pos - traj[idx_1].pos) / (traj[idx_2].timestamp - traj[idx_1].timestamp) / 2;
    }
}

std::string state2str(const JointState &state, int precision)
{
    std::string str = "";
    str += "pos:" + vec2str(state.pos, precision);
    str += " vel:" + vec2str(state.vel, precision);
    str += " torque:" + vec2str(state.torque, precision);
    str += " gripper_pos:" + std::to_string(state.gripper_pos);
    str += " timestamp:" + std::to_string(state.timestamp);
    str += "\n";
    return str;
}

bool JointStateInterpolator::is_initialized()
{
    return _initialized;
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
