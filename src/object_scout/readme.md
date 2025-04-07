# Object Scout

A ROS package for autonomous navigation and object detection using the Locobot platform. This package builds on the yolo_vision package to implement computer vision-based autonomous exploration and object interaction.

## Overview

The object_scout package enables a robot to:
- Autonomously navigate an environment
- Search for specific objects using YOLO-based vision
- Approach detected objects
- Record and manage poses for navigation

## Prerequisites

- ROS Noetic
- Python 3
- OpenCV
- Ultralytics YOLO
- Interbotix ROS packages (for Locobot)
- Dependencies:
  - actionlib
  - geometry_msgs
  - move_base_msgs
  - nav_msgs
  - rospy
  - std_msgs
  - tf2_ros
  - visualization_msgs

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/anramz29/lips_ws.git
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:
```bash
source devel/setup.bash
```

## Package Structure

```
.
├── CMakeLists.txt
├── config
│   └── poses.yaml
├── launch
│   ├── pick_up_object_test.launch
│   ├── place_object_test.launch
│   ├── scout_system_distributed.launch
│   ├── scout_system_locobot.launch
│   └── test_nav_and_scanner.launch
├── package.xml
├── readme.md
├── scripts
│   ├── pickup_object_test_node.py
│   ├── place_object_test_node.py
│   ├── scout_coordinator_locobot_node.py
│   └── test_navigation_node.py
├── setup.py
└── src
    ├── __init__.py
    └── object_scout
        ├── __init__.py
        ├── fine_approacher.py
        ├── navigation_controller.py
        ├── object_approacher.py
        ├── object_scanner.py
        ├── pick_up_object.py
        ├── place_object.py
        ├── pose_manager.py
        └── scout_coordinator_locobot.py

```

## Usage

### Running the Scout System on Locobot

For Locobot, first start the robot's navigation stack:

```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav_2.launch robot_model:=locobot_wx250s use_lidar:=true localization:=true
```

Then launch the scout system (preferably in your remote pc):

```bash
roslaunch object_scout scout_system_locobot.launch
```

#### Note: This is specifically for the locobot due to the fact that on the remote computer we cannot obtain `JointGroupCommand` to tilt the camera servo or manupilate the robot arm.

### Testing Navigation

To test navigation functionality:

```bash
roslaunch object_scout test_nav_and_scanner.launch
```

### Testing Without `Fine_Approacher`

For Locobot, first start the robot's navigation stack:

```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav_2.launch robot_model:=locobot_wx250s use_lidar:=true localization:=true
```

Then launch the scout system on your remote pc:

```bash
roslaunch object_scout scout_system_remote.launch
```

## Configuration

You can customize robot behaviors by modifying parameters in the `config/poses.yaml` file, which contains predefined poses for the robot to navigate to.

## Features

- Autonomous environment exploration
- Object detection and recognition using YOLO vision
- Object approach and interaction
- Pose recording and management
- Integration with navigation systems
- Picking object using it keypoints to change yaw angle of the 6 dof arm

## License

This repository is open-source and available under the MIT License.

## Maintainer

- [Adrián Noé Ramírez](https://github.com/anramz29)