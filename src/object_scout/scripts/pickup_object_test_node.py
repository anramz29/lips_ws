#!/usr/bin/env python3

import rospy
import os
from object_scout.pick_up_object import PickUpObject
from interbotix_xs_msgs.msg import JointGroupCommand # type: ignore

def tilt_camera_directly(robot_name, angle):
    """Tilt camera directly using JointGroupCommand publisher"""
    camera_pub = rospy.Publisher(
        f'/{robot_name}/commands/joint_group', 
        JointGroupCommand, 
        queue_size=1
    )
    
    # Wait for publisher to connect
    rospy.sleep(0.5)
    
    # Create and send message
    camera_msg = JointGroupCommand()
    camera_msg.name = 'camera'
    camera_msg.cmd = [0.0, angle]
    camera_pub.publish(camera_msg)
    
    rospy.loginfo(f"Camera tilt command sent: {angle} radians")
    # Give time for the command to take effect
    rospy.sleep(2.0)

if __name__ == "__main__":

    # Initialize the ROS node
    rospy.init_node('pickup_object_test', anonymous=True)
    
    # Get parameters from the parameter server
    robot_name = rospy.get_param('~robot_name', 'locobot')
    keypoint_angle_topic = rospy.get_param('~keypoint_angle_topic', f'/{robot_name}/keypoint_angle')
    enable_keypoint_detection_service = rospy.get_param('~enable_keypoint_detection_service', 
                                                      f'/{robot_name}/keypoint_detector/set_enabled')
    
    rospy.loginfo(f"Using robot_name: {robot_name}")
    rospy.loginfo(f"Using keypoint_angle_topic: {keypoint_angle_topic}")
    rospy.loginfo(f"Using enable_keypoint_detection_service: {enable_keypoint_detection_service}")
    
    # Create an instance of PickUpObject - pass None for fine_approacher in test mode
    object_picker = PickUpObject(
        robot_name=robot_name,
        fine_approacher=None,  # Not needed for direct testing
        keypoint_angle_topic=keypoint_angle_topic,
        enable_keypoint_detection_service=enable_keypoint_detection_service,
        init_node=False
    )
    
    # Tilt the camera directly to a specified angle (e.g., 0.75 radians)
    tilt_camera_directly(robot_name, 0.75)
    
    # Attempt to pick up an object
    if object_picker.pick_object_with_retries():
        rospy.loginfo("Object picked up successfully")
    else:
        rospy.logwarn("Failed to pick up object")

