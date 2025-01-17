# Trouble Shooting the Ros2 setup

This Troubleshooting Documentation list all the issues i've encountered and how to solve them. 

## Issues:


#### Base Topics are not being Published when I run the xslocobot_control.launch.py file:

- 1. ~~I fogot to do sudo apt update, upgrade and autoremove?~~
- 2. ~~Maybe I need the **create3_ros_ws**?~~
- 3. ~~There are nesseary configurations for a multi-robot setup. Therefore I will turn off other bases and try to run the robot from that.~~
- 4. ~~With the control launch file add the parameter `use_base_odom_tf` to true~~
- 5. Ensure that we all configs are working right.


Once I reinstalled the AMD I ran the [Quick Start Guide](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/quickstart.html)

```bash
ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true
```
The Relavent output was:
```bash
[component_container-6] [INFO] [1737050521.808722383] [tf_rebroadcaster_container]: Found class: rclcpp_components::NodeFactoryTemplate<interbotix_tf_tools::TFRebroadcaster>
[component_container-6] [INFO] [1737050521.808775338] [tf_rebroadcaster_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<interbotix_tf_tools::TFRebroadcaster>
[component_container-6] [INFO] [1737050521.813842042] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_link', prepending prefix 'locobot/'.
[component_container-6] [INFO] [1737050521.813916790] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_footprint', prepending prefix 'locobot/'.
[component_container-6] [INFO] [1737050521.816666954] [tf_rebroadcaster]: Will broadcast TFs from topic '/locobot/mobile_base/tf' to the 'tf' topic under namespace '/'.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/tf_rebroadcaster' in container '/tf_rebroadcaster_container'
```

#### When I tried running:
```bash
ros2 topic pub --rate 10 --times 30 /locobot/mobile_base/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
```
The base didn't move, upon further inspection using:
```bash
ros2 topic list
```
The Output was 
```bash
/locobot/camera/color/camera_info
/locobot/camera/color/image_raw
/locobot/camera/color/image_raw/compressed
/locobot/camera/color/image_raw/compressedDepth
/locobot/camera/color/metadata
/locobot/camera/depth/camera_info
/locobot/camera/depth/color/points
/locobot/camera/depth/image_rect_raw
/locobot/camera/depth/image_rect_raw/compressed
/locobot/camera/depth/image_rect_raw/compressedDepth
/locobot/camera/depth/metadata
/locobot/camera/extrinsics/depth_to_color
/locobot/camera/imu
/locobot/commands/joint_group
/locobot/commands/joint_single
/locobot/commands/joint_trajectory
/locobot/dynamixel/joint_states
/locobot/joint_states
/locobot/mobile_base/joint_states
/locobot/mobile_base/tf
/locobot/robot_description
/parameter_events
/rosout
/tf
/tf_static
```

### Solution (not tested!):

Follow the [RMW_configs](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/rmw_configuration.html) very closely, ensure IP Routing Service and IP Forwarding Enabled. Once these configs are set [resart_rmw](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/troubleshooting.html#can-t-see-topics-published-by-locobot-on-remote-using-ros-2) as seen in the documentation.

#### Rplidar Issues:
```bash
[sllidar_node-4] [ERROR] [1737050524.076826728] [locobot.rplidar_composition]: Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! 
[ERROR] [sllidar_node-4]: process has died [pid 2558, exit code 255, cmd '/home/locobot/interbotix_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node --ros-args -r __node:=rplidar_composition -r __ns:=/locobot --params-file /tmp/launch_params_obchhhiz'].
```