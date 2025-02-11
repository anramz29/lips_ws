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

Contains the implementation of real-time object detection and distance estimation for Locobot.
- [YOLO Vision Documentation](yolo_vision_ROS1/readme.md)
  - Object detection using YOLO
    - [Launch Files](yolo_vision_ROS1/launch/)
    - [Scripts](yolo_vision_ROS1/scripts/)
    - [Models](yolo_vision_ROS1/models/)
  - Distance estimation using depth camera
  - RViz visualization tools
    - [RViz Configurations](yolo_vision_ROS1/rviz/)

---

### Archive ROS1
Contains archived resources, issues, and quick setup for ROS1.
- **Issues**
  - [Display Forwarding Issues](ROS1/Issues/Display_Forwarding_Issues.md)
  - [Navigation Stack Issues](ROS1/Issues/Navigation_Stack_Issues.md)
  - [Rviz Frame Odometry Issues](ROS1/Issues/Rviz_Frame_Odometry_Issues.md)
- [Quick Setup for ROS1](ROS1/quick_setup_Ros1.md)
- **ROS Docker (ROS1)**
  - [Dockerfile](ROS1/Docker/Dockerfile)
  - [Docker Tips and Tricks](ROS1/Docker/Docker_Tips_and_Tricks.md)
  - [Server Documentation for ROS1](ROS1/Docker/Server_Documentation_Ros1.md)
  - [VS Code and Docker](ROS1/Docker/VS_Code_and_Docker.md)

---

### ROS2 Integration (Failed)
Contains resources and documentation for ROS2 integration with Locobot. This integration attempt failed due to RMW (ROS Middleware) configuration incompatibilities between the Intel NUC and iRobot Create3 base, preventing proper communication between components.

- **Docker Info**
  - [Dockerfile](ROS2_Integration_Failed/docker_info/Dockerfile)
  - [Readme](ROS2_Integration_Failed/docker_info/README.md)
  - [Server Documentation for ROS2](ROS2_Integration_Failed/docker_info/Server_Documentation_Ros2.md)
  - [Xslocobot Remote Install Script](ROS2_Integration_Failed/docker_info/xslocobot_remote_install.sh)
- [Quick Setup for ROS2](ROS2_Integration_Failed/quick_setup_ros2.md)
- [Reinstallation Guide](ROS2_Integration_Failed/reinstallation.md)
- [Troubleshooting Guide](ROS2_Integration_Failed/troubleshooting.md)

---
## How to Use This Repository
1. Navigate to the relevant section based on your needs:
   - **ROS1 Users:** Refer to the `ROS1` folder.
   - **ROS2 Users:** Refer to the `ROS2_Integration_Failed` folder.
2. Review the Quick Setup Guides for instructions on setting up Locobot for either ROS1 or ROS2.
3. For Docker-based setups, refer to the respective Docker Info sections for ROS1 and ROS2.
4. For the yolo_vision package, use svn to just copy the respective directory

```bash
svn export https://github.com/anramz29/Locobot_Docs/trunk/yolo_vision_ROS1
```
---

## Contributing

If you find any errors or have suggestions for improvements, please submit a pull request or open an issue.

---
## License
This repository is open-source and available under the MIT License.