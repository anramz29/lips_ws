# Locobot Documentation

By: Adrián Noé Ramírez

This repository contains resources, documentation, and scripts for working with Locobot using ROS1 and ROS2. Below is an overview of the folder structure and their contents, with proper links to all files.

---

## Latest Work

Checkout the [scouting script](/src/object_scout/src/object_scout/scout_coordinator_locobot.py) and [pick up object script](/src/object_scout/src/object_scout/pick_up_object.py) for my latest work in autonomus navigation & 6 dof arm manipulation using computer vision.

<div align="center">
  <a href="https://youtu.be/a55GlM6ha9g">
    <img src="images/robotics_pick_and_place_thumbnail.jpeg" alt="Robotics Pick and Place Demo" width="800"/>
  </a>
  <p><em>Robotics Pick and Place system demonstration (Click on the image!)</em></p>
</div>

The [Object Scout Package](/src/object_scout/src/object_scout/) works as seen in the flowchart below.

<div align="center">
  <img src="images/locobot_flow_chart.png" alt="locobot flow chart" width="800"/>
  <p><em>pick and place flow chart</em></p>
</div>

---

## Other Demos

<div align="center">
  <img src="images/Yolo_Vision.gif" alt="YOLO Vision Demo Video" width="800"/>
  <p><em>YOLO Vision in action (Demo)</em></p>
</div>

<div align="center">
  <img src="images/object_mapper.png" alt="Object Mapper Demo Video" width="800"/>
  <p><em>Object Mapper (Demo)</em></p>
</div>


---

## Directory Structure

### [ROS1](ROS1)
Contains resources, issues, dockerfile, and quick setup for ROS1.

---

### [ROS2 Integration (Failed)](ROS2_Integration_Failed)

Contains resources and documentation for ROS2 integration with Locobot. This integration attempt failed due to RMW (ROS Middleware) configuration incompatibilities between the Intel NUC and iRobot Create3 base, preventing proper communication between components.

---

### [src](src)

Contains packages:
- [object_scout](/src/object_scout/readme.md): Builds off yolo vision for computer vision based autonomus navigation
- [yolo_vision](/src/yolo_vision/): A package that uses computer vision to find an object, find it's distance from the robot, and map the object within the robot map.
- [move_to_pose](/src/move_to_pose/): The package that controls pose recording
- [video_recorder](/src/video_recorder/): Another helper package that takes in the feed of the robot camera, and records a video (used to create single frames to train the computer vision model)

#### for more information vist the respetive [src](/src/readme.md) readme

## How to Use This Repository
1. Navigate to the relevant section based on your needs:
   - **ROS1 Users:** Refer to the `ROS1` folder.
   - **ROS2 Users:** Refer to the `ROS2_Integration_Failed` folder.
2. Review the Quick Setup Guides for instructions on setting up Locobot for either ROS1 or ROS2.
3. For Docker-based setups, refer to the respective Docker Info sections for ROS1 and ROS2.
4. For the `src` packages, use a git to just copy the respective directory, then run a `catkin_make`


## Installation

1. Clone this repository into your catkin workspace:
```bash
git clone https://github.com/anramz29/lips_ws.git
```

2. Build the package:
```bash
cd ~/lips_ws
catkin_make
```

3. Source your workspace:
```bash
source devel/setup.bash
```
---

## Contributing

If you find any errors or have suggestions for improvements, please submit a pull request or open an issue.

---

## Maintainer

- [Adrián Noé Ramírez](mailto:anramz29@gmail.com)

---
## License
This repository is open-source and available under the MIT License.

