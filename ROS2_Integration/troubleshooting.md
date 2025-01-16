# Trouble Shooting the Ros2 setup

This Troubleshooting Documentation list all the issues i've encountered and how to solve them. 

## Issues:


#### Base Issues:

- 1. ~~I fogot to do sudo apt update, upgrade and autoremove?~~
- 2. ~~Maybe I need the **create3_ros_ws**?~~
- 3. ~~There are nesseary configurations for a multi-robot setup. Therefore I will turn off other bases and try to run the robot from that.~~
- 4. ✅ With the control launch file add the parameter `use_base_odom_tf` to true 


Once I reinstalled the AMD [Quick Start Guide](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/quickstart.html)

```bash
ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true
```
The output was:
```bash
[INFO] [launch]: All log files can be found below /home/locobot/.ros/log/2025-01-16-12-02-00-421137-locobot-2534
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [realsense2_camera_node-5]: process started with pid [2560]
[INFO] [robot_state_publisher-1]: process started with pid [2552]
[INFO] [joint_state_publisher-2]: process started with pid [2554]
[INFO] [xs_sdk-3]: process started with pid [2556]
[INFO] [sllidar_node-4]: process started with pid [2558]
[INFO] [component_container-6]: process started with pid [2562]
[sllidar_node-4] [INFO] [1737050521.575471504] [locobot.rplidar_composition]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1, SLLIDAR SDK Version:2.1.0
[xs_sdk-3] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-3] [INFO] Using logging level 'INFO'.
[xs_sdk-3] [INFO] Loaded mode configs from '/home/locobot/interbotix_ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/modes_all.yaml'.
[xs_sdk-3] [INFO] Loaded motor configs from '/home/locobot/interbotix_ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/locobot_wx250s.yaml'.
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID: 11, Model: 'XL430-W250-2', Joint Name: 'tilt'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  8, Model: 'XL430-W250', Joint Name: 'wrist_rotate'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  7, Model: 'XM430-W350', Joint Name: 'wrist_angle'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  5, Model: 'XM430-W350', Joint Name: 'elbow_shadow'.
[component_container-6] [INFO] [1737050521.802846274] [tf_rebroadcaster_container]: Load Library: /home/locobot/interbotix_ws/install/interbotix_tf_tools/lib/libtf_rebroadcaster.so
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  4, Model: 'XM430-W350', Joint Name: 'elbow'.
[component_container-6] [INFO] [1737050521.808722383] [tf_rebroadcaster_container]: Found class: rclcpp_components::NodeFactoryTemplate<interbotix_tf_tools::TFRebroadcaster>
[component_container-6] [INFO] [1737050521.808775338] [tf_rebroadcaster_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<interbotix_tf_tools::TFRebroadcaster>
[component_container-6] [INFO] [1737050521.813842042] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_link', prepending prefix 'locobot/'.
[component_container-6] [INFO] [1737050521.813916790] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_footprint', prepending prefix 'locobot/'.
[component_container-6] [INFO] [1737050521.816666954] [tf_rebroadcaster]: Will broadcast TFs from topic '/locobot/mobile_base/tf' to the 'tf' topic under namespace '/'.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/tf_rebroadcaster' in container '/tf_rebroadcaster_container'
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  6, Model: 'XM430-W350', Joint Name: 'forearm_roll'.
[realsense2_camera_node-5] [INFO] [1737050521.870513245] [locobot.camera.camera]: RealSense ROS v4.51.1
[realsense2_camera_node-5] [INFO] [1737050521.871141143] [locobot.camera.camera]: Built with LibRealSense v2.51.1
[realsense2_camera_node-5] [INFO] [1737050521.871500118] [locobot.camera.camera]: Running with LibRealSense v2.51.1
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  2, Model: 'XM430-W350', Joint Name: 'shoulder'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID: 10, Model: 'XL430-W250-2', Joint Name: 'pan'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  9, Model: 'XL430-W250', Joint Name: 'gripper'.
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  3, Model: 'XM430-W350', Joint Name: 'shoulder_shadow'.
[realsense2_camera_node-5] [INFO] [1737050522.035426522] [locobot.camera.camera]: Device with serial number 213322072455 was found.
[realsense2_camera_node-5] 
[realsense2_camera_node-5] [INFO] [1737050522.035482735] [locobot.camera.camera]: Device with physical ID /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video0 was found.
[realsense2_camera_node-5] [INFO] [1737050522.035497674] [locobot.camera.camera]: Device with name Intel RealSense D435 was found.
[realsense2_camera_node-5] [INFO] [1737050522.035724967] [locobot.camera.camera]: Device with port number 2-1 was found.
[realsense2_camera_node-5] [INFO] [1737050522.035748343] [locobot.camera.camera]: Device USB type: 3.2
[realsense2_camera_node-5] [INFO] [1737050522.037306279] [locobot.camera.camera]: getParameters...
[realsense2_camera_node-5] [INFO] [1737050522.038121105] [locobot.camera.camera]: JSON file is not provided
[realsense2_camera_node-5] [INFO] [1737050522.038162062] [locobot.camera.camera]: Device Name: Intel RealSense D435
[realsense2_camera_node-5] [INFO] [1737050522.038372960] [locobot.camera.camera]: Device Serial No: 213322072455
[realsense2_camera_node-5] [INFO] [1737050522.038392257] [locobot.camera.camera]: Device physical port: /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/video4linux/video0
[realsense2_camera_node-5] [INFO] [1737050522.038409132] [locobot.camera.camera]: Device FW version: 05.12.07.150
[realsense2_camera_node-5] [INFO] [1737050522.038423033] [locobot.camera.camera]: Device Product ID: 0x0B07
[realsense2_camera_node-5] [INFO] [1737050522.038436677] [locobot.camera.camera]: Sync Mode: Off
[xs_sdk-3] [INFO] 	Found DYNAMIXEL ID:  1, Model: 'XM430-W350', Joint Name: 'waist'.
[realsense2_camera_node-5] [WARN] [1737050522.153697064] [locobot.camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-5] [WARN] [1737050522.159747834] [locobot.camera.camera]: Could not set param: rgb_camera.power_line_frequency with 3 Range: [0, 2]: parameter 'rgb_camera.power_line_frequency' could not be set: Parameter {} doesn't comply with integer range.
[realsense2_camera_node-5]  16/01 12:02:02,160 WARNING [139948472653568] (backend-v4l2.cpp:1444) Pixel format 36315752-1a66-a242-9065-d01814a likely requires patch for fourcc code RW16!
[realsense2_camera_node-5] [WARN] [1737050522.166595848] [locobot.camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-5] [INFO] [1737050522.176426293] [locobot.camera.camera]: Stopping Sensor: Stereo Module
[realsense2_camera_node-5] [INFO] [1737050522.236941057] [locobot.camera.camera]: Starting Sensor: Stereo Module
[realsense2_camera_node-5] [INFO] [1737050522.239481267] [locobot.camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 640, Height: 480, FPS: 30
[realsense2_camera_node-5] [INFO] [1737050522.239922458] [locobot.camera.camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-5] [INFO] [1737050522.290733024] [locobot.camera.camera]: Starting Sensor: RGB Camera
[realsense2_camera_node-5] [INFO] [1737050522.296026122] [locobot.camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 30
[realsense2_camera_node-5] [INFO] [1737050522.297373759] [locobot.camera.camera]: RealSense Node Is Up!
[realsense2_camera_node-5] [WARN] [1737050522.358992182] [locobot.camera.camera]: 
[xs_sdk-3] [WARN] Writing startup register values to EEPROM. This only needs to be done once on a robot if using a default motor config file, or after a motor config file has been modified. Can set `write_eeprom_on_startup` to false from now on.
[xs_sdk-3] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'time'.
[xs_sdk-3] [INFO] The operating mode for the 'camera' group was changed to 'position' with profile type 'velocity'.
[xs_sdk-3] [INFO] The operating mode for the 'gripper' joint was changed to 'pwm' with profile type 'velocity'.
[xs_sdk-3] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-3] [INFO] [1737050523.776700493] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
[sllidar_node-4] [ERROR] [1737050524.076826728] [locobot.rplidar_composition]: Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! 
[ERROR] [sllidar_node-4]: process has died [pid 2558, exit code 255, cmd '/home/locobot/interbotix_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node --ros-args -r __node:=rplidar_composition -r __ns:=/locobot --params-file /tmp/launch_params_obchhhiz'].
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

#### Solution:

You need to specify to rebroadcast the base transform to the locobot with the `use_odom_base_tf`:=param

```bash
ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true use_base_odom_tf:=true
```

```bash
ros2 topic list
```
The Output was:
```bash
locobot@locobot:~$ ros2 topic list
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
/locobot/mobile_base/battery_state
/locobot/mobile_base/cliff_intensity
/locobot/mobile_base/cmd_audio
/locobot/mobile_base/cmd_lightring
/locobot/mobile_base/cmd_vel
/locobot/mobile_base/dock
/locobot/mobile_base/hazard_detection
/locobot/mobile_base/imu
/locobot/mobile_base/interface_buttons
/locobot/mobile_base/ir_intensity
/locobot/mobile_base/ir_opcode
/locobot/mobile_base/joint_states
/locobot/mobile_base/kidnap_status
/locobot/mobile_base/mobility_monitor/transition_event
/locobot/mobile_base/mouse
/locobot/mobile_base/odom
/locobot/mobile_base/robot_state/transition_event
/locobot/mobile_base/slip_status
/locobot/mobile_base/static_transform/transition_event
/locobot/mobile_base/stop_status
/locobot/mobile_base/tf
/locobot/mobile_base/tf_static
/locobot/mobile_base/wheel_status
/locobot/mobile_base/wheel_ticks
/locobot/mobile_base/wheel_vels
/locobot/robot_description
/parameter_events
/rosout
/tf
/tf_static
```

#### Rplidar Issues:
```bash
[sllidar_node-4] [ERROR] [1737050524.076826728] [locobot.rplidar_composition]: Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! 
[ERROR] [sllidar_node-4]: process has died [pid 2558, exit code 255, cmd '/home/locobot/interbotix_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node --ros-args -r __node:=rplidar_composition -r __ns:=/locobot --params-file /tmp/launch_params_obchhhiz'].
```