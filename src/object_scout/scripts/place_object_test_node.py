#!/usr/bin/env python3

import rospy
import os
from object_scout.place_object import PlaceObject
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
    rospy.init_node('place_object_test', anonymous=True)
    
    # Get parameters
    robot_name = rospy.get_param('~robot_name', 'locobot')
    
    object_placer = PlaceObject(robot_name, init_node=False)
    
    # Tilt the camera directly to a specified angle (e.g., 1.5 radians)
    tilt_camera_directly(robot_name, .75)

    # Attempt to place an object
    if object_placer.place_at_bbox_center():
        rospy.loginfo("Object placed successfully")
    else:
        rospy.logwarn("Failed to place object")
