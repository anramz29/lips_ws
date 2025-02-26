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
        self.max_approach_retries = rospy.get_param('~max_approach_retries', 2)
        self.retry_delay = rospy.get_param('~retry_delay', 3.0)  # seconds
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize components
        self.nav_controller = NavigationController(self.robot_name)
        self.pose_manager = PoseManager(self.poses_config)
        self.scanner = ObjectScanner(self.robot_name, self.nav_controller)
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller)
        
        # Tracking state for fallback behavior
        self.approach_attempts = 0
        
    def move_to_named_pose(self, pose_name):
        """
        Move to a named pose and scan for objects
        
        Args:
            pose_name: Name of the pose in configuration
            
        Returns:
            tuple: (success, approach_attempted, approach_succeeded)
            - success: Whether navigation and scanning succeeded
            - approach_attempted: Whether approach was attempted
            - approach_succeeded: Whether approach succeeded
        """
        # Get current robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False, False, False
            
        # Get target pose from pose manager
        target_pose = self.pose_manager.get_pose(pose_name)
        if target_pose is None:
            return False, False, False
        
        # Move to final destination
        rospy.loginfo(f"Moving to position: {pose_name}")
        success = self.nav_controller.move_to_position(
            target_pose.position.x,
            target_pose.position.y
        )
        
        if not success:
            rospy.logwarn(f"Failed to reach {pose_name}")
            return False, False, False
            
        # Scan at destination
        rospy.loginfo(f"Scanning at position: {pose_name}")
        detection = self.scanner.perform_scan_rotation()
        
        # If object detected, attempt approach
        if detection == ScanResult.OBJECT_DETECTED:
            rospy.loginfo(f"Object detected at {pose_name}, initiating approach...")
            approach_result = self.approacher.approach_object(self.scanner.object_marker)
            
            if approach_result:
                rospy.loginfo(f"Successfully approached object at {pose_name}")
                return True, True, True
            else:
                rospy.logwarn(f"Failed to approach object at {pose_name}: {self.approacher.get_failure_reason()}")
                return True, True, False
        
        return True, False, False
        
    def move_to_all_poses(self):
        """
        Move through all poses, scanning and approaching if objects found.
        If approach fails, continue with remaining poses.
        
        Returns:
            bool: True if object found and successfully approached, False otherwise
        """
        all_pose_names = self.pose_manager.get_all_pose_names()
        successful_approach = False
        
        # Set to track poses we've visited
        visited_poses = set()
        
        # Try to visit all poses
        pose_index = 0
        
        while pose_index < len(all_pose_names) and not rospy.is_shutdown():
            pose_name = all_pose_names[pose_index]
            
            # Skip if we've already visited this pose
            if pose_name in visited_poses:
                pose_index += 1
                continue
                
            rospy.loginfo(f"\nMoving to pose: {pose_name} ({pose_index + 1}/{len(all_pose_names)})")
            success, approach_attempted, approach_succeeded = self.move_to_named_pose(pose_name)
            
            # Mark this pose as visited
            visited_poses.add(pose_name)
            
            # Handle approach results
            if approach_attempted:
                if approach_succeeded:
                    # Successfully approached object!
                    successful_approach = True
                    rospy.loginfo(f"Successfully approached object at {pose_name}")
                    break  # End the mission on successful approach
                else:
                    # Approach failed, but we should continue with other poses
                    rospy.logwarn(f"Approach failed at {pose_name}, continuing with remaining poses")
                    
            # Move to next pose
            pose_index += 1
            
            # If we've reached the end without success, end the mission
            if pose_index >= len(all_pose_names):
                rospy.loginfo("Completed all poses, no successful object approach")
                
            rospy.sleep(1)  # Brief pause between movements
            
        return successful_approach
        
    def main(self):
        """
        Main execution function
        
        Returns:
            bool: Success flag indicating if an object was found
        """
        rospy.loginfo("=== Object Scout Mission Started ===")
        result = self.move_to_all_poses()
        
        if result:
            rospy.loginfo("Successfully completed mission and approached an object!")
        else:
            rospy.loginfo("Completed mission, no objects successfully approached")
            
        return result


if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinator(init_node=True)
        result = coordinator.main()
        if not result:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")