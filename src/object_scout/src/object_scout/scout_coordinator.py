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
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize components
        self.nav_controller = NavigationController(self.robot_name)
        self.pose_manager = PoseManager(self.poses_config)
        self.scanner = ObjectScanner(self.robot_name, self.nav_controller)
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller)
        
    def move_to_named_pose(self, pose_name):
        """
        Move to a named pose with intermediate scanning
        
        Args:
            pose_name: Name of the pose in configuration
            
        Returns:
            bool: Success flag
        """
        # Get current robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False
            
        # Get target pose from pose manager
        target_pose = self.pose_manager.get_pose(pose_name)
        if target_pose is None:
            return False
        
        # Move to final destination
        rospy.loginfo(f"Moving to final position: {pose_name}")
        success = self.nav_controller.move_to_position(
            target_pose.position.x,
            target_pose.position.y
        )
        
        if success:
            # Scan at final destination
            rospy.loginfo(f"Scanning at final position: {pose_name}")
            detection = self.scanner.perform_scan_rotation()
            
            # If object detected, approach it
            if detection == ScanResult.OBJECT_DETECTED:
                # Maximum number of approach attempts
                max_attempts = 3
                current_attempt = 1
                approach_success = False
                
                # Try approaching until successful or max attempts reached
                while current_attempt <= max_attempts and not approach_success:
                    rospy.loginfo(f"Approach attempt {current_attempt}/{max_attempts}")
                    
                    # Try to approach the object
                    approach_success = self.approacher.approach_object(self.scanner.object_marker)
                    
                    if approach_success:
                        # Handle successful approach
                        rospy.loginfo(f"Successfully approached object on attempt {current_attempt}")
                        # Add any follow-up actions here
                        return True
                    else:
                        # Handle failed approach
                        rospy.logwarn(f"Failed to approach object on attempt {current_attempt}")
                        
                        if current_attempt < max_attempts:
                            # Reset detection state before attempting the offset scan
                            self.scanner.reset_detection_state()
                            
                            # Calculate offset angle (45 degrees × attempt number)
                            offset_angle = 45 * current_attempt
                            rospy.loginfo(f"Trying offset scan with {offset_angle} degree rotation")
                            
                            # Resume scan with calculated offset
                            offset_detection = self.scanner.resume_scan_with_offset(offset_degrees=offset_angle)
                            
                            if offset_detection != ScanResult.OBJECT_DETECTED:
                                rospy.logwarn("No object detected during offset scan")
                                # Exit the loop if we no longer detect the object
                                break
                        
                    current_attempt += 1
                
                rospy.logwarn(f"Failed to approach object after {max_attempts} attempts")
                return False
            else:
                rospy.loginfo("No object detected at final position")
                return True
        else:
            return False
        
    def move_to_all_poses(self):
        """
        Move through all poses, scanning and approaching if objects found
        
        Returns:
            bool: True if object found, False otherwise
        """
        found_object = False
        
        for pose_name in self.pose_manager.get_all_pose_names():
            if found_object:
                rospy.loginfo("Object already found and approached, ending navigation sequence")
                break
                
            rospy.loginfo(f"\nMoving to: {pose_name}")
            success = self.move_to_named_pose(pose_name)
            
            # If success and object detected by scanner, set flag
            if success and self.scanner.object_detected:
                found_object = True
                rospy.loginfo(f"Object detected at {pose_name}, stopping navigation sequence")

            if not success:
                rospy.logwarn(f"Failed to reach {pose_name}, continuing to next pose...")
                
            rospy.sleep(1)  # Brief pause between movements
            
        return found_object
        
    def main(self):
        """
        Main execution function
        
        Returns:
            bool: Success flag indicating if an object was found
        """
        rospy.loginfo("=== Object Scout Mission Started ===")
        result = self.move_to_all_poses()
        
        if result:
            rospy.loginfo("Successfully completed mission and found an object!")
        else:
            rospy.loginfo("Completed mission, no objects approached")
            
        return result


if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinator(init_node=True)
        result = coordinator.main()
        if not result:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")