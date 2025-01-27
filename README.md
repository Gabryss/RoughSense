A ROS2-compatible package for estimating terrain roughness using LiDAR and IMU data. 
This repository provides tools to analyze and quantify environmental roughness, facilitating navigation and traversability assessments for robotics applications. 

Ideal for autonomous systems that rely on robust perception in challenging terrains.

## Usage
Clone this repository within a ROS2 workspace. Assuming you have named your workspace `ros2_ws` and placed it under your home folder:
```
$ cd ~/ros2_ws/src/
$ git clone https://github.com/Gabryss/RoughSense.git
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws/
$ colcon build
$ source install/setup.bash
$ ros2 launch RoughSense roughness_launch.py
```

A config file is provided within `/config` to fine tune the algorithm according to your robot needs.

## In progress
- Roughness prediction corrected with IMU values.
- Dockerization
