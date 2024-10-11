#include "app/joint_controller.h"
#include <chrono>
#include <csignal>
#include <unistd.h>

using namespace arx;

Arx5JointController *arx5_joint_controller = new Arx5JointController("X5", "can0", "../models/arx5.urdf");

void signal_handler(int signal)
{
    std::cout << "SIGINT received" << std::endl;
    delete arx5_joint_controller;
    exit(signal);
}

int main()
{
    std::signal(SIGINT, signal_handler);
    int loop_cnt = 0;
    while (true)
    {
        JointState state = arx5_joint_controller->get_state();
        Pose6d pose6 = arx5_joint_controller->get_tool_pose();
        std::cout << "Gripper: " << state.gripper_pos << ", joint pos: " << state.pos.transpose()
                  << ", Pose: " << pose6.transpose() << std::endl;
        usleep(10000); // 10ms
        loop_cnt++;
        if (loop_cnt % 200 == 0)
        {
            arx5_joint_controller->reset_to_home();
            arx5_joint_controller->set_to_damping();
        }
    }
    return 0;
}