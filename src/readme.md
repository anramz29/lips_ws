# lips_ws Packages!

This comprises the packages that are currently being used in the lips lab. Currently finished the preliminary [search alogrithim](/src/move_to_pose/launch/scan_and_approach.launch)

Also Note that these scripts are ran on a remote pc and are meant to be used with a launch driver. In this case im using the [interbotix nav stack](https://github.com/Interbotix/interbotix_ros_rovers/tree/main/interbotix_ros_xslocobots/interbotix_xslocobot_nav) or the [interbotix control package](https://github.com/Interbotix/interbotix_ros_rovers/tree/main/interbotix_ros_xslocobots/interbotix_xslocobot_control)


for example on the locobot I run:
```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav_2.launch 
robot_model:=locobot_wx250s use_lidar:=true localization:=true
```

### **Important**
If your not using a locobot check the topics in the launch scripts and adjust the launch file to your specific specs


<div align="center">
  <img src="../images/object_approach.png" alt="Object Mapper Demo Video" width="800"/>
  <p><em>Object Approach (Demo)</em></p>
</div>
    

## Prerequisites

- ROS (tested on ROS Noetic)
- Python 3
- OpenCV
- Ultralytics YOLO
- CvBridge
- Sensor Messages
- Geometry Messages
- **Interbotix ROS packages**



## Structure
```
.
├── move_to_pose
│   ├── config
│   │   └── poses.yaml
│   ├── launch
│   │   └── move_to_pose_scan.launch
│   ├── scripts
│   │   ├── goal_recorder.py
│   │   └── move_to_pose_node.py
│   ├── setup.py
│   └── src
│       ├── __init__.py
│       └── move_to_pose_utils.py
├── object_scout
│   ├── config
│   │   └── poses.yaml
│   ├── launch
│   │   ├── scout_system.launch
│   │   └── test_nav_and_scanner.launch
│   ├── scripts
│   │   ├── scout_coordinator_node.py
│   │   └── test_navigation_node.py
│   ├── setup.py
│   └── src
│       ├── __init__.py
│       └── object_scout
│           ├── __init__.py
│           ├── navigation_controller.py
│           ├── object_approacher.py
│           ├── object_scanner.py
│           ├── pose_manager.py
│           ├── scout_coordinator.py
│           └── utils.py
├── readme.md
├── video_recorder
│   ├── launch
│   │   └── video_recorder.launch
│   ├── photos
│   │   └── back_table.jpg
│   ├── scripts
│   │   ├── picture_snapper.py
│   │   └── video_recorder.py
│   └── videos
│       └── video_20250126_163047.mp4
└── yolo_vision
    ├── include
    │   ├── distance_node.hpp
    │   └── yolo_node.hpp
    ├── launch
    │   ├── object_mapper.launch
    │   ├── yolo_distance.launch
    │   └── yolo_vision.launch
    ├── models
    │   ├── best.pt
    │   └── yolo11n.pt
    ├── rviz
    │   └── xslocobot_description_yolo.rviz
    ├── scripts
    │   ├── distance_node.py
    │   ├── object_mapper_node.py
    │   ├── search_and_approach_node.py
    │   └── yolo_node.py
    └── src
        ├── distance_node.cpp
        └── yolo_node.cpp
```

