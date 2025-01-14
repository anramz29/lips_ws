# LoCoBot Control and Object Detection Setup

### Date Jan 13th 2025

## Prerequisites
- ROS environment configured on both robot and remote computer
- LoCoBot WX250S hardware
- Network connection between robot and remote computer

## On the Robot

### 1. Launch Robot Control
Open a terminal and run:
```bash
roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s use_base:=true use_camera:=true
```

This command:
- Initializes the LoCoBot control system
- Enables base movement
- Activates the camera

### 2. Launch Joystick Control
Open a new terminal and run:
```bash
roslaunch interbotix_xslocobot_joy xslocobot_joy.launch robot_model:=locobot_wx250s launch_driver:=false
```

This command:
- Enables joystick control
- Uses existing driver (hence `launch_driver:=false`)

## On the Remote Computer

### Launch Object Detection
Open a terminal and run:
```bash
roslaunch find_object_2d find_object_2d.launch image_topic:=/locobot/camera/color/image_raw
```

This command:
- Starts the object detection node
- Subscribes to the robot's camera feed
- Opens the object detection GUI

## Verification
- Confirm that all nodes are running without errors
- Check that the camera feed appears in the object detection window
- Test joystick control functionality
