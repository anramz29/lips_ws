#!/usr/bin/env python3

import rospy
import os
from object_scout.pick_up_object import PickUpObject
from object_scout.fine_approacher import FineApproacher
from object_scout.navigation_controller import NavigationController
from object_scout.pose_manager import PoseManager
from interbotix_xs_msgs.msg import JointGroupCommand

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
    # Initialize ROS node
    rospy.init_node('pick_up_object_standalone', anonymous=False)
    robot_name = 'locobot'
    
    try:
        # Get the correct path to poses.yaml
        # First try to use ROS package path
        import rospkg
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path('object_scout')
            poses_config = os.path.join(pkg_path, 'config', 'poses.yaml')
        except rospkg.common.ResourceNotFound:
            # Fallback to hardcoded path if package not found
            poses_config = '/home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/object_scout/config/poses.yaml'
        
        # Make sure the file exists
        if not os.path.exists(poses_config):
            rospy.logwarn(f"Poses config file not found at {poses_config}, using default")
            poses_config = ''  # Empty string will use default poses
        
        # Initialize components
        pose_manager = PoseManager(poses_config=poses_config, init_node=False)
        nav_controller = NavigationController(robot_name, pose_manager, init_node=False)
        
        # Tilt camera directly - use a higher value for better view
        # 0.75 radians is about 43 degrees down
        tilt_camera_directly(robot_name, 0.75)
        
        # Create instance of the PickUpObject class - use proper parameters
        pickup = PickUpObject(robot_name=robot_name, init_node=False)
        
        # Run the pick operation
        result = pickup.pick_object()
        
        if result:
            rospy.loginfo("Successfully picked up object!")
        else:
            rospy.logwarn("Failed to pick up object")
            
        # Reset camera tilt before finishing
        tilt_camera_directly(robot_name, 0.2618)  # Default tilt (15 degrees)
        
        # Keep the node running until shutdown
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Error occurred: {e}")