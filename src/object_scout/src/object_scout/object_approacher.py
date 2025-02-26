#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
from object_scout.navigation_controller import NavigationController
from object_scout.object_scanner import ObjectScanner, ScanResult
from object_scout.object_approacher import ObjectApproacher
from object_scout.pose_manager import PoseManager
from object_scout.utils import get_robot_pose


class RobustScoutCoordinator:
    """
    Enhanced coordinator for the object scouting mission with improved fallback behavior
    """
    def __init__(self, init_node=True):
        """
        Initialize the coordinator
        
        Args:
            init_node: Whether to initialize a ROS node
        """
        if init_node:
            rospy.init_node('robust_scout_coordinator', anonymous=False)
        
        # Get ROS parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        self.max_approach_retries = rospy.get_param('~max_approach_retries', 2)
        self.retry_delay = rospy.get_param('~retry_delay', 3.0)  # seconds
        self.min_detection_confidence = rospy.get_param('~min_detection_confidence', 0.6)
        self.fallback_poses_enabled = rospy.get_param('~fallback_poses_enabled', True)
        self.max_fallback_retries = rospy.get_param('~max_fallback_retries', 3)
        
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
        self.fallback_attempts = 0
        self.current_fallback_pose_index = 0
        self.fallback_poses = {}
        
    def initialize_fallback_poses(self):
        """
        Initialize fallback poses for each primary pose
        
        Each primary pose will have a list of nearby alternative poses to try
        if the sustained detection fails at the primary pose
        """
        all_poses = self.pose_manager.get_all_pose_names()
        
        # For each primary pose, create a list of fallback poses
        for pose_name in all_poses:
            # Get primary pose
            primary_pose = self.pose_manager.get_pose(pose_name)
            if primary_pose is None:
                continue
                
            # Create fallback poses around the primary pose
            # These are slightly offset positions to try if detection fails
            fallbacks = []
            
            # Create a grid of fallback positions around the primary pose
            offsets = [(-0.3, 0), (0.3, 0), (0, -0.3), (0, 0.3), 
                      (-0.3, -0.3), (-0.3, 0.3), (0.3, -0.3), (0.3, 0.3)]
            
            for i, (x_offset, y_offset) in enumerate(offsets):
                fallback_name = f"{pose_name}_fallback_{i+1}"
                fallback_pose = self.pose_manager.create_pose(
                    fallback_name,
                    primary_pose.position.x + x_offset,
                    primary_pose.position.y + y_offset,
                    primary_pose.orientation
                )
                fallbacks.append(fallback_name)
                
            self.fallback_poses[pose_name] = fallbacks
            
        rospy.loginfo(f"Initialized fallback poses for {len(self.fallback_poses)} primary poses")
        
    def move_to_named_pose(self, pose_name, is_fallback=False):
        """
        Move to a named pose and scan for objects with enhanced fallback
        
        Args:
            pose_name: Name of the pose in configuration
            is_fallback: Whether this is a fallback pose
            
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
            rospy.logerr(f"Failed to get pose: {pose_name}")
            return False, False, False
        
        # Move to final destination
        pose_type = "fallback" if is_fallback else "primary"
        rospy.loginfo(f"Moving to {pose_type} position: {pose_name}")
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
        else:
            rospy.loginfo(f"No objects detected at {pose_name}")
            
        return True, False, False
        
    def try_fallback_poses(self, primary_pose_name):
        """
        Try fallback poses for a primary pose where detection failed
        
        Args:
            primary_pose_name: The primary pose that failed
            
        Returns:
            tuple: (approach_attempted, approach_succeeded)
            - approach_attempted: Whether approach was attempted
            - approach_succeeded: Whether approach succeeded
        """
        if not self.fallback_poses_enabled:
            rospy.loginfo("Fallback poses disabled, skipping fallbacks")
            return False, False
            
        fallbacks = self.fallback_poses.get(primary_pose_name, [])
        if not fallbacks:
            rospy.logwarn(f"No fallback poses available for {primary_pose_name}")
            return False, False
            
        rospy.loginfo(f"Trying {len(fallbacks)} fallback poses for {primary_pose_name}")
        
        for i, fallback_name in enumerate(fallbacks):
            if self.fallback_attempts >= self.max_fallback_retries:
                rospy.logwarn(f"Reached maximum fallback retry limit ({self.max_fallback_retries})")
                break
                
            rospy.loginfo(f"Trying fallback pose {i+1}/{len(fallbacks)}: {fallback_name}")
            success, approach_attempted, approach_succeeded = self.move_to_named_pose(
                fallback_name, is_fallback=True
            )
            
            self.fallback_attempts += 1
            
            if approach_attempted:
                if approach_succeeded:
                    rospy.loginfo(f"Successfully approached object from fallback pose {fallback_name}")
                    return True, True
                else:
                    rospy.logwarn(f"Approach failed from fallback pose {fallback_name}")
            
            if not success:
                rospy.logwarn(f"Failed to navigate to fallback pose {fallback_name}")
                
            # Brief pause between fallback attempts
            rospy.sleep(1.0)
            
        rospy.loginfo(f"Completed all fallback poses for {primary_pose_name} without success")
        return True, False
        
    def move_to_all_poses(self):
        """
        Move through all poses, scanning and approaching if objects found.
        Uses fallback poses when detection or approach fails.
        
        Returns:
            bool: True if object found and successfully approached, False otherwise
        """
        all_pose_names = self.pose_manager.get_all_pose_names()
        successful_approach = False
        
        # Initialize fallback poses
        self.initialize_fallback_poses()
        
        # Set to track poses we've visited
        visited_poses = set()
        visited_fallbacks = set()
        
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
                    # Primary approach failed, could try fallbacks but continue for now
                    # Fallbacks are typically for detection failures
                    rospy.logwarn(f"Approach failed at {pose_name}, continuing with remaining poses")
            elif success and not approach_attempted:
                # Detection failed, try fallback poses
                rospy.loginfo(f"No detection at {pose_name}, trying fallback poses")
                fallback_attempted, fallback_succeeded = self.try_fallback_poses(pose_name)
                
                if fallback_succeeded:
                    # Successfully approached from a fallback pose!
                    successful_approach = True
                    rospy.loginfo(f"Successfully approached object using fallback pose for {pose_name}")
                    break  # End the mission on successful approach
                elif fallback_attempted:
                    rospy.logwarn("All fallback approaches failed, continuing with remaining poses")
                else:
                    rospy.loginfo("No fallbacks were attempted, continuing with remaining poses")
                    
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
        rospy.loginfo("=== Robust Object Scout Mission Started ===")
        result = self.move_to_all_poses()
        
        if result:
            rospy.loginfo("Successfully completed mission and approached an object!")
        else:
            rospy.loginfo("Completed mission, no objects successfully approached")
            
        return result


if __name__ == "__main__":
    try:
        coordinator = RobustScoutCoordinator(init_node=True)
        result = coordinator.main()
        if not result:
            rospy.loginfo("Mission complete without success. Keeping node alive...")
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Robust scout coordinator interrupted")