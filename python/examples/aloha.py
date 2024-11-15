from doctest import master
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
@click.argument("master_model")  # ARX arm model: X5 or L5
@click.argument("master_interface")  # can bus name (can0 etc.)
@click.argument("slave_model")  # ARX arm model: X5 or L5
@click.argument("slave_interface")  # can bus name (can0 etc.)
def main(master_model: str, master_interface: str, slave_model: str, slave_interface: str):
    np.set_printoptions(precision=3, suppress=True)
    assert(master_interface != slave_interface)

    master = arx5.Arx5JointController(master_model, master_interface)
    slave = arx5.Arx5JointController(slave_model, slave_interface)
    robot_config = master.get_robot_config()
    controller_config = master.get_controller_config()

    master.reset_to_home()
    slave.reset_to_home()

    gain = arx5.Gain(robot_config.joint_dof)
    gain.kd()[:] = 0.01
    master.set_gain(gain)
    try:
        while True:
            joint_state = master.get_joint_state()
            joint_state.timestamp = 0
            joint_state.torque()[:] = 0.0
            joint_state.vel()[:] *= 0.3  # add some speed for lower latency. 
            # You can set it to 0.0 if you only want to use position control
            joint_state.gripper_torque = 0.0
            joint_state.gripper_vel = 0.0
            slave.set_joint_cmd(joint_state)
            time.sleep(controller_config.controller_dt)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Resetting arms to home position...")
        slave.reset_to_home()
        master.reset_to_home()
        print("Arms reset to home position. Exiting.")


    



main()
