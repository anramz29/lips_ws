# Vision-Based Action Selection for a ROS Robot

## Overview
This research project aims to develop a robot system utilizing **computer vision** and **ROS** for autonomous decision-making. The project focuses on enabling the robot to measure its environment, identify "interesting" elements, and select appropriate actions to engage with these elements. The development is currently in its initial stage, building on the **ros_navigation** package and incorporating object detection and basic pathing functionalities.

---

## Key Questions

### Action Selection Architecture

- **Objectives of the Robot:**

#### **Fold 1:**
- Initial goals include simple exploration, object identification, and interaction with the environment, along with efficient pathing.

- **Decision-Making Process:**

#### **Fold 1:**
  - The robot will select future actions based on the **move_base** node as defined below.
  - There will be no specific rules or algorithms for prioritizing tasks at this stage.

- **Criteria for Success:**
  - **Fold 1:** Accurate identification of objects within the environment.
  - **Fold 2** Incorporating efficient pathing and handling environmental challenges to evaluate the robustness of the CV algorithm.

---

### Defining "Interesting"

#### **Fold 1:**
- The focus will be on basic object detection using the **rtabmap_ros** package and the **find_object_2d** tool.

#### **Fold 2:**
- **Definition:**
  - "Interesting" elements are features or objects that meet specific criteria defined by the task.

- **Examples:**
  - Objects with specific features such as shapes, colors, or motion.
  - High-entropy regions for exploration and foraging tasks.

- **Implementation Approach:**
  - Train the computer vision algorithm on specific features (e.g., a toy bug).
  - Use RTAB-Map and/or **cv_bridge** to implement a simple ML algorithm.

---

## ROS Implementation

### Open Source ROS Packages Utilized

#### **interbotix_xslocobot_nav**:
   - Based on the **interbotix_xslocobot_control** package, the **interbotix_xslocobot_nav** incorporates the **rtabmap_ros** package, which facilitates simultaneous localization and mapping (SLAM) using camera data. It supports localization and mapping functionalities.

#### Nodes within: **interbotix_xslocobot_nav**

1. **RGB-D Synchronization Node (`rgbd_sync`)**:
   - Synchronizes color and depth data from the RealSense camera.
   - Key Parameters: `approx_sync` (false), decimation factor, voxel size.
   - **Important**: The RealSense camera uses a launch driver from the [IntelRealSense](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) repository.

2. **Obstacle Detection Node (`obstacle_detection`)**:
   - Identifies ground and obstacle points using RGB-D data.

3. **RTAB-Map Node (`rtabmap`)**:
   - Handles core SLAM functionality, publishing transform (TF) data for odometry and map frames.
   - Key Parameters:
     - `subscribe_rgbd`: Enables RGB-D subscription.
     - `database_path`: Path for saving the RTAB-Map database.
     - `tag_variances`: Angular and linear variances for odometry TFs.

4. **RTAB-Map Visualization (`rtabmapviz`)**:
   - Visualizes the map and localization data.

5. **Move Base (`move_base`)**:
   - Responsible for global and local planning using costmaps.
   - Loads configurations for local and global costmaps, planners, and move base parameters.

#### Nodes that need to be added

1.  **Find Object Node (`find_object_2d`)**

   - link: http://wiki.ros.org/find_object_2d
   - This package provides a simple Qt interface to try OpenCV implementations of SIFT, SURF, FAST, BRIEF, and other feature detectors and descriptors.
   - May require remapping some topics.


---

## Camera Topics

1. **/locobot/camera/color/image_raw**
   - For 2D computer vision, this topic could be used as input for a custom 2D CNN.

---

## Current Steps

1. Integrate **find_object_2d** into the **interbotix_xslocobot_nav** launch file.
2. Ensure that object detection messages are being published in RViz.
3. Build upon this with the **[find_object_3d](https://github.com/introlab/find-object/blob/master/launch/ros1/find_object_3d.launch)** package.

---

## Additional Notes

- **Iterative Development:**
  - Start with simple scenarios to test the CV algorithm’s ability to detect predefined objects.
  - Gradually introduce complexity, such as competing "interesting" regions or challenging environmental conditions.

---

