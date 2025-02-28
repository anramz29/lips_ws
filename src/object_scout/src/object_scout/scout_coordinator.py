#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
from object_scout.navigation_controller import NavigationController
from object_scout.object_scanner import ObjectScanner, ScanResult
from object_scout.object_approacher import ObjectApproacher
from object_scout.pose_manager import PoseManager
from object_scout.utils import get_robot_pose


class ScoutCoordinator:
    """
    Main coordinator for the object scouting mission
    """
    def __init__(self, init_node=True):
        """
        Initialize the coordinator
        
        Args:
            init_node: Whether to initialize a ROS node
        """
        if init_node:
            rospy.init_node('scout_coordinator', anonymous=False)
        
        # Get ROS parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        self.max_objects = rospy.get_param('~max_objects', 0)  # 0 means find all objects
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize components
        self.nav_controller = NavigationController(self.robot_name)
        self.pose_manager = PoseManager(self.poses_config)
        self.scanner = ObjectScanner(self.robot_name, self.nav_controller)
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller)
        
        # Track found objects
        self.found_objects = []



if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinator(init_node=True)
        num_objects = coordinator.main()
        if num_objects == 0:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")