# Darknet_ROS Workspace Build Guide

This guide walks you through building the Darknet_ROS workspace and configuring its launch files for custom settings. For full reference, visit the official [darknet_ros GitHub repository](https://github.com/leggedrobotics/darknet_ros).

------
# Preview

<video width="640" height="360" controls>
  <source src="/home/adramirez/Locobot_Docs/images_and_videos/Computer_Vision_Robotics.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

## Building the Darknet_ROS Workspace

1. **Clone the Repository:**
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/leggedrobotics/darknet_ros.git
   ```

2. **Install Dependencies:**
   Ensure you have all required ROS and system dependencies installed. Run the following:
   ```bash
   sudo apt-get update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace:**
   Navigate to the root of your catkin workspace and build the project:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source the Workspace:**
   Source your workspace to make the `darknet_ros` package available:
   ```bash
   source devel/setup.bash
   ```

## Launch File Configuration

### Navigating to Launch Files

The launch files for `darknet_ros` are located in the `launch` directory of the package:
```bash
cd ~/catkin_ws/src/darknet_ros/darknet_ros/launch
```

### Modifying the `image` Argument

To configure the camera input for your specific setup, modify the `image` argument in the launch file. For example, in a typical launch file:

```xml
<launch>
  <!-- Console launch prefix -->
  <arg name="launch_prefix" default=""/>
  <arg name="image" default="locobot/camera/color/image_raw" />

  <!-- Config and weights folder. -->
  <arg name="yolo_weights_path" default="$(find darknet_ros)/yolo_network_config/weights"/>
  <arg name="yolo_config_path" default="$(find darknet_ros)/yolo_network_config/cfg"/>

  <!-- ROS and network parameter files -->
  <arg name="ros_param_file" default="$(find darknet_ros)/config/ros.yaml"/>
  <arg name="network_param_file" default="$(find darknet_ros)/config/yolov2-tiny.yaml"/>

  <!-- Load parameters -->
  <rosparam command="load" ns="darknet_ros" file="$(arg ros_param_file)"/>
  <rosparam command="load" ns="darknet_ros" file="$(arg network_param_file)"/>

  <!-- Start darknet and ros wrapper -->
  <node pkg="darknet_ros" type="darknet_ros" name="darknet_ros" output="screen" launch-prefix="$(arg launch_prefix)">
    <param name="weights_path" value="$(arg yolo_weights_path)" />
    <param name="config_path" value="$(arg yolo_config_path)" />
    <remap from="camera/rgb/image_raw" to="$(arg image)" />
  </node>
</launch>
```

### Steps to Update the Argument

1. Open the desired launch file using your preferred text editor:
   ```bash
   nano darknet_ros.launch
   ```

2. Locate the following argument:
   ```xml
   <arg name="image" default="locobot/camera/color/image_raw" />
   ```

3. Change the `default` value to match your camera topic. For example, if your camera topic is `/camera/rgb/image_raw`, update it as follows:
   ```xml
   <arg name="image" default="/camera/rgb/image_raw" />
   ```

4. Save the file and exit the editor.

### Running the Launch File

Once the configuration is updated, you can run the launch file:
```bash
roslaunch darknet_ros darknet_ros.launch
```

This will start the `darknet_ros` node with the configured settings.

---

For additional customization, refer to the [darknet_ros documentation](https://github.com/leggedrobotics/darknet_ros).



