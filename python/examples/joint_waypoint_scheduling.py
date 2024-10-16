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
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5.urdf", help="URDF file path")
def main(model: str, interface: str, urdf_path: str):
    controller = arx5.Arx5JointController(model, interface, urdf_path)
    controller.set_log_level(arx5.LogLevel.DEBUG)
    controller.reset_to_home()
    joint_waypoints = np.array(
        [[1.0, 2.0, 2.0, 1.5, 1.5, -1.57], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    )
    waypoint_interval_s = 5.0
    for waypoint in joint_waypoints:
        joint_state = controller.get_joint_state()
        joint_cmd = arx5.JointState(waypoint, np.zeros(6), np.zeros(6), 0.0)
        joint_cmd.timestamp = joint_state.timestamp + waypoint_interval_s
        controller.set_joint_cmd(joint_cmd)
        time.sleep(waypoint_interval_s)

    time.sleep(1.0)
    controller.reset_to_home()


if __name__ == "__main__":
    main()
