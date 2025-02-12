# Locobot Documentation

By: Adrián Noé Ramírez

This repository contains resources, documentation, and scripts for working with Locobot using ROS1 and ROS2. Below is an overview of the folder structure and their contents, with proper links to all files.

---
## Directory Structure
### YOLO Vision (ROS 1)


<div align="center">
  <img src="yolo_vision_ROS1/images/Yolo_Vision.gif" alt="YOLO Vision Demo Video" width="800"/>
  <p><em>YOLO Vision in action (Demo)</em></p>
</div>

<div align="center">
  <img src="yolo_vision_ROS1/images/object_mapper.png" alt="Object Mapper Demo Video" width="800"/>
  <p><em>Object Mapper (Demo)</em></p>
</div>

Contains the implementation of real-time object detection and distance estimation for Locobot.
- [YOLO Vision Documentation](yolo_vision_ROS1/readme.md)
  - Object detection using YOLO
    - [Launch Files](yolo_vision_ROS1/launch/yolo_vision.launch)
    - [Scripts](yolo_vision_ROS1/scripts/yolo_node.py)
    - [Models](yolo_vision_ROS1/models/)
  - Distance estimation using depth camera
    - [Launch Files](yolo_vision_ROS1/launch/yolo_distance.launch)
    - [Scripts](yolo_vision_ROS1/scripts/distance_node.py)
  - Object Mapping
    - [Launch Files](yolo_vision_ROS1/launch/object_mapper.launch)
    - [Scripts](yolo_vision_ROS1/scripts/object_mapper_node.py)
  - RViz visualization tools
    - [RViz Configurations](yolo_vision_ROS1/rviz/)

---

### [ROS1](ROS1)
Contains resources, issues, dockerfile, and quick setup for ROS1.

---

### [ROS2 Integration (Failed)](ROS2_Integration_Failed)

Contains resources and documentation for ROS2 integration with Locobot. This integration attempt failed due to RMW (ROS Middleware) configuration incompatibilities between the Intel NUC and iRobot Create3 base, preventing proper communication between components.

---
## How to Use This Repository
1. Navigate to the relevant section based on your needs:
   - **ROS1 Users:** Refer to the `ROS1` folder.
   - **ROS2 Users:** Refer to the `ROS2_Integration_Failed` folder.
2. Review the Quick Setup Guides for instructions on setting up Locobot for either ROS1 or ROS2.
3. For Docker-based setups, refer to the respective Docker Info sections for ROS1 and ROS2.
4. For the `yolo_vision_Ros1` package, use svn to just copy the respective directory

```bash
svn export https://github.com/anramz29/Locobot_Docs/trunk/yolo_vision_ROS1
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

