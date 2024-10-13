import time
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import numpy as np


joint_dof = 7
solver = arx5.Arx5Solver(
    # "../models/arx5_gopro.urdf",
    "../models/arx7_left.urdf",
    joint_dof,
    # "base_link",
    # "eef_link",
    # np.array([0, 0, -9.807], dtype=np.float64),
)

# solver = arx5.Arx5Solver("../models/arx5_gopro.urdf", joint_dof)

# joint_dof = 7
# solver = arx5.Arx5Solver(
#     "../models/arx7_left.urdf",
#     joint_dof,
#     "base_link",
#     "link7",
#     np.array([0, 0, -9.807]),
# )

print("Done initialization")

joint_pos = np.zeros(joint_dof)
joint_pos[2] = np.pi / 2
joint_pos[3] = -np.pi / 2
joint_vel = np.zeros(joint_dof)
joint_acc = np.zeros(joint_dof)
fk_pose = solver.forward_kinematics(joint_pos)

print(f"{fk_pose=}")

succeed, ik_joint_pos = solver.inverse_kinematics(fk_pose, joint_pos)

print(f"{ik_joint_pos=}")

start_time = time.monotonic()
id_torque = solver.inverse_dynamics(joint_pos, joint_vel, joint_acc)
print(f"{id_torque=}")
