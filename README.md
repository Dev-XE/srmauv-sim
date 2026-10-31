# SRM AUV Simulation

This repository contains the SRM AUV simulation package for ROS 2 and
Gazebo Classic.

## Software Versions

- ROS 2 Humble
- Gazebo Classic 11.10.2

## Running the Simulator

From a terminal, run the following commands:

```bash
cd <cloned-folder-name>
colcon build
source install/setup.bash
ros2 launch srmauv_description gazebo.launch.py
```


