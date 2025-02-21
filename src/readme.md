# catkin_ws

This comprises the packages that are currently being used in the lips lab. Currently working on a search and approach algorythim.

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
    

## Structure
```
.
├── move_to_pose
│   ├── config
│   │   └── poses.yaml
│   ├── launch
│   │   ├── approach_object.launch
│   │   ├── find_object.launch
│   │   ├── move_to_pose.launch
│   │   └── move_to_pose_scan.launch
│   ├── scripts
│   │   ├── approach_object.py
│   │   ├── find_object.py
│   │   ├── goal_recorder.py
│   │   ├── move_to_pose_node.py
│   │   └── move_to_pose_scan_node.py
│   ├── setup.py
│   └── src
│       ├── __init__.py
│       └── move_to_pose_utils.py
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
    ├── launch
    │   ├── object_mapper.launch
    │   ├── search_and_approach.launch
    │   ├── yolo_distance.launch
    │   └── yolo_vision.launch
    ├── models
    │   ├── best.pt
    │   └── yolo11n.pt
    ├── rviz
    │   └── xslocobot_description_yolo.rviz
    └── scripts
        ├── distance_node.py
        ├── object_mapper_node.py
        ├── search_and_approach_node.py
        └── yolo_node.py

15 directories, 29 files
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

## Installation

1. Clone this repository into your catkin workspace:
```bash
git clone https://github.com/anramz29/lips_ws
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

# Topics

## Video Recorder Topics

### Camera Topics
- `rgb_topic`: RGB image raw topic 
  - Default: `/locobot/camera/color/image_raw`
  - Source: Color camera raw image stream


## Move_To_Pose Topics

### Move Base Topics
- Move Base Action Server
  - Topic: `/locobot/move_base`
  - Type: `MoveBaseAction`
  - Purpose: Enables navigation and movement commands for the robot
  - Allows sending goal poses for the robot to navigate to

### Costmap Topics
- Global Costmap Topic
  - Topic: `/locobot/move_base/global_costmap/costmap`
  - Type: `OccupancyGrid`
  - Purpose: Provides a 2D grid representing obstacles and traversability
  - Used for path planning and obstacle avoidance
  - Values:
    - 0: Free space
    - 100: Completely occupied
    - -1: Unknown/unobserved space

### Yolo Topics Connected to the Package
- `object_marker_topic`: Topic for object markers in RViz
  - Default: `object_markers`
  - Publishes marker visualizations of detected objects
- `bbox_depth_topic`: Topic for bounding box depth information
  - Default: `camera/yolo/bbox_depth`
  - Publishes detected object bounding boxes with depth information

---

## Yolo_Vision Topics

### Global Arguments
- `robot_name`: Default namespace for all nodes (default: "locobot")
- `rviz_frame`: Frame used in RViz (default: "map")
- `rvizconfig`: RViz configuration file path

### Camera Topics
- `rgb_topic`: RGB image raw topic 
  - Default: `/locobot/camera/color/image_raw`
  - Source: Color camera raw image stream

- `depth_topic`: Depth image topic
  - Default: `/locobot/camera/depth/image_rect_raw`
  - Source: Depth camera rectified image stream

### YOLO Detection Topics
- `model_path`: Path to the YOLO model file
  - Default: `$(find yolo_vision)/models/best.pt`
  - Used for object detection configuration

- `annotated_image_topic`: Topic for YOLO annotated images
  - Default: `camera/yolo/annotated_image`
  - Publishes images with bounding boxes drawn by YOLO

- `bbox_depth_topic`: Topic for bounding box depth information
  - Default: `camera/yolo/bbox_depth`
  - Publishes detected object bounding boxes with depth information

- `visualization_topic`: Topic for distance visualization
  - Default: `camera/yolo/distance_image`
  - Publishes visualizations of object distances

### Additional Topics
- `camera_info_topic`: Camera calibration information topic
  - Default: `camera/color/camera_info`
  - Provides camera intrinsic parameters

- `object_marker_topic`: Topic for object markers in RViz
  - Default: `object_markers`
  - Publishes marker visualizations of detected objects
