import time

import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click
import numpy as np


@click.command()
@click.argument("leader_model")  # ARX arm model: X5 or L5
@click.argument("leader_interface")  # can bus name (can0 etc.)
@click.argument("follower_model")  # ARX arm model: X5 or L5
@click.argument("follower_interface")  # can bus name (can0 etc.)
def main(leader_model: str, leader_interface: str, follower_model: str, follower_interface: str):
    np.set_printoptions(precision=3, suppress=True)
    assert(leader_interface != follower_interface)

    leader = arx5.Arx5JointController(leader_model, leader_interface)
    follower = arx5.Arx5JointController(follower_model, follower_interface)
    robot_config = leader.get_robot_config()
    assert robot_config.joint_dof == follower.get_robot_config().joint_dof
    controller_config = leader.get_controller_config()

    leader.reset_to_home()
    follower.reset_to_home()

    gain = arx5.Gain(robot_config.joint_dof)
    gain.kd()[:] = 0.01
    leader.set_gain(gain)
    try:
        while True:
            leader_joint_state = leader.get_joint_state()
            follower_joint_cmd = arx5.JointState(robot_config.joint_dof)
            follower_joint_cmd.pos()[:] = leader_joint_state.pos()
            follower_joint_cmd.gripper_pos = leader_joint_state.gripper_pos
            
            # If you want to decrease the delay of teleoperation , you can uncomment the following line
            # This will partially include the velocity to the command, and you will need to pass these velocities
            # into the policy layout
            # follower_joint_cmd.vel()[:] = leader_joint_state.vel()* 0.3 
            
            follower.set_joint_cmd(follower_joint_cmd)
            time.sleep(controller_config.controller_dt)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Resetting arms to home position...")
        follower.reset_to_home()
        leader.reset_to_home()
        print("Arms reset to home position. Exiting.")


    



main()
