#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg
from geometry_msgs.msg import PoseStamped

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

from src.object_scout.navigation_controller import NavigationController

# Custom service message imports (you would need to define these)
# from object_scout.srv import MoveToPosition, MoveToPositionResponse

def move_to_position_cb(req):
    """
    Service callback for move_to_position service
    
    Note: You would need to define the appropriate service message
    """
    # Extract parameters from the request
    x = req.x
    y = req.y
    
    # Call the controller method
    success = controller.move_to_position(x, y)
    
    # Return the response
    return {'success': success}

def move_to_pose_cb(req):
    """
    Service callback for handling PoseStamped messages
    
    Note: You would need to define the appropriate service message
    """
    # Extract parameters from the request
    pose = req.pose
    
    # Call the controller method
    success = controller.move_to_position(
        pose.position.x, 
        pose.position.y,
        pose.orientation
    )
    
    # Return the response
    return {'success': success}

def main():
    # Initialize the node
    rospy.init_node('navigation_controller')
    
    # Get parameters
    robot_name = rospy.get_param('~robot_name', 'locobot')
    
    # Create the controller
    global controller
    controller = NavigationController(robot_name, init_node=False)
    
    # Set up services
    # Note: You would need to define and build these service messages
    # rospy.Service('~move_to_position', MoveToPosition, move_to_position_cb)
    # rospy.Service('~move_to_pose', MoveToPose, move_to_pose_cb)
    
    # Log startup
    rospy.loginfo("Navigation Controller Node started")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation controller node interrupted")