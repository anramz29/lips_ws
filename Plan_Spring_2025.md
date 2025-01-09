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

## Additional Notes

- **Iterative Development:**
  - Start with simple scenarios to test the CV algorithm’s ability to detect predefined objects.
  - Gradually introduce complexity, such as competing "interesting" regions or challenging environmental conditions.

- **Future Directions:**
  - Explore reinforcement learning for adaptive action selection.
  - Integrate advanced pathing algorithms for efficiency in navigation.
  - Test the system’s performance in dynamic, real-world environments.

---

