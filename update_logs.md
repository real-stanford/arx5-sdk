## Update (2026.01.29)
- Update library for x86_64 Ubuntu20.04. If you have trouble loading the current libraries (version GLIBC_2.32 not found), please override `lib/x86_64/libsolver_20_04.so` to `lib/x86_64/libsolver.so` and `lib/x86_64/libhardware_20_04.so` to `lib/x86_64/libhardware.so` and recompile the package.
- The default libraries are compiled on Ubuntu 22.04

## Update (2025.08.29)
- Support gripper force control (see `python/examples/test_gripper_force_compensation.py`)
- Set conda-forge::soem version to 1.4.0 in conda environment files, thanks [Haoyu Xiong](https://haoyu-x.github.io/) for pointing out.
- If you encounter error `undefined symbol: EcatError`, this is because the soem version is updated to 2.0.0, which is not compatible with the current version of the controller. Please downgrade soem version to 1.4.0 (`conda install conda-forge::soem=1.4.0`).

## Update (2024.12.05)
- Add safety checks to zmq_server
- Unify the joint interpolator in both joint controller and cartesian controller for better smoothness
- Support trajectory updating and velocity interpolation
- Fix various bugs for gravity compensation, robot initialization etc.

## Update (2024.08.22)
- Enable one-step waypoint scheduling (see `python/examples`).
- Support EtherCAT-CAN adapter (follow the instructions in [EtherCAT-CAN setup](README.md#ethercat-can-setup)).
- Support arbitrary DoF robot arm (not only 6DoF); thoguh other DoF numbers are not tested yet.
- Allow setting up robot and controller configurations as arguments (see `config.h` and `test_joint_control.py`), thanks to [Yifan Hou](https://yifan-hou.github.io/)

When updating the sdk to your codebase, please first remove the entire `build` folder and run the building process again.