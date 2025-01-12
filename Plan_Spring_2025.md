# Vision-Based Action Selection for a ROS Robot

## Overview
This research project focuses on developing a robot system using **computer vision** and **ROS** to enable autonomous decision-making. The robot's primary tasks include measuring its environment, extracting "interesting" elements, and selecting appropriate actions to interact with or attend to these elements.

---

## Key Questions

### Action Selection Architecture

- **Objectives of the Robot:**
  - Initial goals include simple exploration and object identification.
  - Future goals include interaction with the environment and efficient pathing.

- **Decision-Making Process:**
  - How does the robot choose the next action?
  - What are the specific rules or algorithms for prioritizing tasks?

- **Criteria for Success:**
  - **Simple Stage:** Accurately identifying predefined objects (e.g., a toy bug).
  - **Advanced Stage:** Incorporating efficient pathing and environmental challenges to evaluate the robustness of the CV algorithm.

---

### Defining "Interesting"

- **Definition:**
  - "Interesting" elements are features or objects that meet specific criteria defined by the task.
  
- **Examples:**
  - Objects with specific features such as shapes, colors, or motion.
  - High-entropy regions for exploration and foraging tasks.

- **Implementation Approach:**
  - Train the computer vision algorithm on specific features (e.g., the toy bug).
  - Specfically Use R
  - Deploy the robot to search the environment for objects matching these features.

---

### Controller Convergence

- **Key Question:**
  - How do you determine when a controller (e.g., visual servoing, motion) has converged?

- **Indicators of Convergence:**
  - The robot successfully centers its focus or reaches the desired object/location.
  - Minimal changes in the control parameters over time.
  - Feedback mechanisms confirm goal achievement (e.g., a reduction in error metrics).

---

### Task Switching

- **Key Question:**
  - What conditions cause the robot to stop the current task and move to another?

- **Criteria for Task Switching:**
  - The current task has been completed (e.g., object identified or goal reached).
  - A higher-priority event is detected (e.g., a new "interesting" region).
  - Time constraints or environmental changes necessitate re-prioritization.

---

## Implementation

### Open Source ROS Packages Utilized

#### **interbotix_xslocobot_nav**:
   - Based on the **interbotix_xslocobot_control** package, it incorporates the **rtabmap_ros** package, which facilitates simultaneous localization and mapping (SLAM) using camera data. It supports localization and mapping functionalities.

#### RTAB-Map (Real-Time Appearance-Based Mapping)
   - The RTAB-Map framework is used for real-time SLAM, leveraging the **rtabmap_ros** package for integration with ROS.

---

### Configuration and Parameters

The RTAB-Map implementation is configured using a comprehensive ROS launch file. Below are key configurable arguments and their purposes:

#### Launch Arguments
- `robot_model`: Specifies the robot model.
- `robot_name`: Default is `locobot`, used to name the robot namespace.
- `use_lidar`: Toggles the use of LiDAR for mapping.
- `base_type`: Sets the base type (e.g., `create3`, `kobuki`).
- `localization`: Enables or disables localization mode.
- `rtabmap_args`: Specifies additional RTAB-Map parameters.
- `camera_tilt_angle`: Sets the tilt angle of the camera (default: 0.2618 radians).

#### RTAB-Map Parameters
Depending on whether LiDAR is used, different sets of parameters are applied to optimize mapping and localization:

**With LiDAR**:
- Neighbor link refining enabled (`RGBD/NeighborLinkRefining` = true).
- Proximity detection enabled (`RGBD/ProximityBySpace` = true).
- Grid ray tracing active.

**Without LiDAR**:
- Depth-based grid mapping active (`Grid/FromDepth` = true).
- Linear and angular updates tuned for RGB-D data.

---

### Node Setup

The launch file initializes multiple nodes and groups for functionality:

#### RTAB-Map Integration
1. **RGB-D Synchronization Node (`rgbd_sync`)**:
   - Synchronizes color and depth data from the RealSense camera.
   - Key Parameters: `approx_sync` (false), decimation factor, voxel size.

2. **Obstacle Detection Node (`obstacle_detection`)**:
   - Identifies ground and obstacle points using RGB-D data.

3. **RTAB-Map Node (`rtabmap`)**:
   - Handles core SLAM functionality, publishing transform (TF) data for odometry and map frames.
   - Key Parameters:
     - `subscribe_rgbd`: Enables RGB-D subscription.
     - `database_path`: Path for saving the RTAB-Map database.
     - `tag_variances`: Angular and linear variances for odometry TFs.

4. **RTAB-Map Visualization (`rtabmapviz`)** (optional):
   - Visualizes the map and localization data.

#### Navigation Nodes
- **Move Base (`move_base`)**:
  - Responsible for global and local planning using costmaps.
  - Loads configurations for local and global costmaps, planners, and move base parameters.

#### Auxiliary Nodes
- **Camera Tilt Control**:
  - Publishes a one-time command to tilt the camera to the specified angle.

---

### Workflow Summary
1. **Startup**:
   - The `interbotix_xslocobot_control` package initializes the hardware.
   - The RTAB-Map nodes are launched with the required parameters.
   - Localization and mapping parameters adapt dynamically based on whether LiDAR is used.

2. **Data Handling**:
   - RGB-D or LiDAR data feeds into RTAB-Map for map generation and localization.
   - Nodes process synchronized data for obstacle detection, localization, and navigation.

3. **Navigation**:
   - The `move_base` node computes paths using global and local planners.
   - The robot autonomously navigates while updating its map using RTAB-Map.

4. **Visualization** (Optional):
   - The `rtabmapviz` node can be enabled for monitoring the robot's mapping and navigation performance.

---





## Additional Notes

- **Iterative Development:**
  - Start with simple scenarios to test the CV algorithm’s ability to detect predefined objects.
  - Gradually introduce complexity, such as competing "interesting" regions or challenging environmental conditions.

- **Future Directions:**
  - Explore reinforcement learning for adaptive action selection.
  - Integrate advanced pathing algorithms for efficiency in navigation.
  - Test the system’s performance in dynamic, real-world environments.

---

