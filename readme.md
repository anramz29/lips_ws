# Locobot Documentation
This repository contains resources, documentation, and scripts for working with Locobot using ROS1 and ROS2. Below is an overview of the folder structure and their contents, with proper links to all files.
---
## Directory Structure
### YOLO Vision (ROS 1)
Contains the implementation of real-time object detection and distance estimation for Locobot.
- [YOLO Vision Documentation](yolo_vision/readme.md)
  - Object detection using YOLO
  - Distance estimation using depth camera
  - RViz visualization tools
---
### Archive ROS1
Contains archived resources, issues, and quick setup for ROS1.
- **Issues**
  - [Display Forwarding Issues](Archive_ROS1/Issues/Display_Forwarding_Issues.md)
    - [Rviz Issue Semi-Solved Image](Archive_ROS1/Issues/images/Rviz_Issue_Semi_Solved.png)
    - [Rviz Odom Issue Image](Archive_ROS1/Issues/images/Rviz_Odom_Issue.png)
  - [Navigation Stack Issues](Archive_ROS1/Issues/Navigation_Stack_Issues.md)
  - [Rviz Frame Odometry Issues](Archive_ROS1/Issues/Rviz_Frame_Odometry_Issues.md)
- [Quick Setup for ROS1](Archive_ROS1/quick_setup_Ros1.md)
- **ROS Docker (ROS1)**
  - [Dockerfile](Archive_ROS1/ROS_Docker/Dockerfile)
  - [Docker Tips and Tricks](Archive_ROS1/ROS_Docker/Docker_Tips_and_Tricks.md)
  - [Server Documentation for ROS1](Archive_ROS1/ROS_Docker/Server_Documentation_Ros1.md)
  - [VS Code and Docker](Archive_ROS1/ROS_Docker/VS_Code_and_Docker.md)
---
### ROS2 Integration (Failed)
Contains resources and documentation for ROS2 integration with Locobot. This integration attempt failed due to RMW (ROS Middleware) configuration incompatibilities between the Intel NUC and iRobot Create3 base, preventing proper communication between components.
- **Docker Info**
  - [Dockerfile](ROS2_Integration/Docker_info/Dockerfile)
  - [Readme](ROS2_Integration/Docker_info/README.md)
  - [Server Documentation for ROS2](ROS2_Integration/Docker_info/Server_Documentation_Ros2.md)
  - [Xslocobot Remote Install Script](ROS2_Integration/Docker_info/xslocobot_remote_install.sh)
- [Quick Setup for ROS2](ROS2_Integration/quick_setup_Ros2.md)
- [Reinstallation Guide](ROS2_Integration/Reinstallation.md)
- [Troubleshooting Guide](ROS2_Integration/Troubleshooting.md)
---
### Root Files
- [Repository Readme](README.md)
---
## How to Use This Repository
1. Navigate to the relevant section based on your needs:
   - **ROS1 Users:** Refer to the `Archive_ROS1` folder.
   - **ROS2 Users:** Refer to the `ROS2_Integration` folder.
2. Review the Quick Setup Guides for instructions on setting up Locobot for either ROS1 or ROS2.
3. For Docker-based setups, refer to the respective Docker Info sections for ROS1 and ROS2.
---
## Contributing
If you find any errors or have suggestions for improvements, please submit a pull request or open an issue.
---
## License
This repository is open-source and available under the MIT License.