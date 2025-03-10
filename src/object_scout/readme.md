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
object_scout/
├── config
│   └── poses.yaml          # Configuration for saved poses
├── launch
│   ├── scout_system.launch        # Main launch file
│   └── test_nav_and_scanner.launch # Testing navigation
├── scripts
│   ├── scout_coordinator_locobot_node.py
│   ├── scout_coordinator_remote_node.py
│   └── test_navigation_node.py
├── setup.py               # Python package setup
└── src
    ├── __init__.py
    └── object_scout
        ├── __init__.py
        ├── navigation_controller.py
        ├── object_approacher.py
        ├── object_scanner.py
        ├── pose_manager.py
        ├── scout_coordinator.py
        └── utils.py
```

## Usage

### Running the Scout System on Locobot

For Locobot, first start the robot's navigation stack:

```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav_2.launch robot_model:=locobot_wx250s use_lidar:=true localization:=true
```

Then launch the scout system:

```bash
roslaunch object_scout scout_system_locobot.launch
```

#### Note: This is specifically for the locobot due to the fact that on the remote computer we cannot obtain `JointGroupCommand` to tilt the camera servo.

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

## License

See the project's license file for details.

## Maintainer

- [Adrián Noé Ramírez](https://github.com/anramz29)