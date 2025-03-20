# lips_ws Packages

This repository contains packages currently used in the LIPS lab. We have implemented a preliminary [search algorithm](/src/object_scout/src/object_scout/scout_coordinator_locobot.py).

**Note:** These scripts are designed to run on a remote PC with a launch driver. We're using either the [Interbotix navigation stack](https://github.com/Interbotix/interbotix_ros_rovers/tree/main/interbotix_ros_xslocobots/interbotix_xslocobot_nav) or the [Interbotix control package](https://github.com/Interbotix/interbotix_ros_rovers/tree/main/interbotix_ros_xslocobots/interbotix_xslocobot_control).

For example, on the locobot:
```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav_2.launch robot_model:=locobot_wx250s use_lidar:=true localization:=true
```

## Prerequisites

- ROS (tested on ROS Noetic)
- Python 3
- OpenCV
- Ultralytics YOLO
- CvBridge
- Sensor Messages
- Geometry Messages
- **Interbotix ROS packages**

# License

This repository is open-source and available under the MIT License.
