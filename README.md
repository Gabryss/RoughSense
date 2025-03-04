A ROS2-compatible package for estimating terrain roughness using a point cloud and IMU data in real time. 
This repository provides tools to analyze and quantify environmental roughness, facilitating navigation and traversability assessments for robotics applications. 

Ideal for autonomous systems that rely on robust perception in challenging terrains.

## Requirements
This ROS2 package rely on ROS2 and has been tested with Humble distribution. Furthermore, it has been tested with the cloud map output from a SLAM algorithm.
Even though a SLAM algorithm is not required to run the algorithm, it is strongly recommended to use it as pairs.

This package require Liquid DSP library for active resonating frequencies filtering.

Installation process
```
$ sudo apt update
$ sudo apt install libliquid-dev
```
To identify the resonating frequencies, multiple data recording have been done with the rover maintaining static while trying to move (with all the sensors active) and on a rough terrain. Then the spectum of both type of experiment was studied to determine which frequencies needs to be filtered.


## Usage
This package has been made and build using ROS2, hence, it require the installation on ROS2 on your device.

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
---


## Configuration
A config file is provided within `/config` to fine tune the algorithm according to your robot needs.

- debug_info: Indicate more information in the terminal [boolean]
- save_data": Save the output data in a directory. Needs a folder called /data in the root workspace containing three folders: /sensors, /pc_vells and /global_maps [boolean]
- debug_time_s":5, [integer]

- pc_topic": Name of the topic to get the point cloud from [string]
- cmd_vel_topic": Name of the topic to get the velocity commands from [string]
- tf_topic": Name of the topic to get the transforms from [string]
- height": 'Height' of the local map [float]

- map_resolution": Resolution of the local and global map (in meters). [float]
- local_map_size": Size of the local map (in meters). [float]
- global_map_size": Size of the global map (in meters). [integer]
- map_unknown_transparency": Unknown cells in white or transparents [boolean]
- map_low_resolution_division_factor": Increase local map resolution (usually between 1 and 4). [integer]

- ransac_iterations": Number of ransac iteration before finding a solution [integer]

- roughness_frame_id": Frame name to publish the roughness [string]
- roughness_topic_global": Topic name of the global map [string]
- roughness_topic_local": Topic name of the local map [string]
- roughness_lidar_threshold": Normalisation threshold for prediction [float]
- roughness_shift": Normalisation element for prediction [float]


- imu_topic": Name of the topic to get the observation from [string]
- imu_sampling_frequency": IMU sampling frequency [integer]
- imu_filter_frequency": Filter frequency [float]
- imu_filter_bandwidth": Width of the filter (in Hz). [float]
- roughness_imu_threshold": Normalisation threshold for observation [float]
- imu_window_size": Sliding window size [integer]
- activate_imu_correction": Activate or deactivate IMU correction [boolean]

- rls_forgeting_factor": Forgeting factor [float]
- activate_idw": Activate or deactivate IDW interpolation [boolean]
- idw_power": IDW interpolation factor [float]

- tf_frequency": TF frequency subscription (in Hz) [float]
- roughness_frequency": Roughness frequency computation and publication (in Hz) [float]



## Global map

The local and global map are composed of 9 layers:
- Roughness predicted raw
- Roughness predicted normalised
- Roughness observed raw
- Roughness observed normalised
- RLS (Alpha)
- RLS (Beta)
- Roughness corrected (using RLS)
- IDW (delta)
- IDW (Roughness corrected + IDW)

At every time `t` the local roughness is computed on every layers and copied in the global map.

Both can be visualised in a classical visualisation software.

## Submission

This project has been submited to the conference IEEE IROS 2025.