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
        
    def move_to_named_pose(self, pose_name):
        """
        Move to a named pose with intermediate scanning
        
        Args:
            pose_name: Name of the pose in configuration
            
        Returns:
            tuple: (success_flag, object_found_flag)
        """
        # Get current robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False, False
            
        # Get target pose from pose manager
        target_pose = self.pose_manager.get_pose(pose_name)
        if target_pose is None:
            return False, False
        
        # Move to final destination
        rospy.loginfo(f"Moving to final position: {pose_name}")
        success = self.nav_controller.move_to_position(
            target_pose.position.x,
            target_pose.position.y
        )
        
        object_found = False
        
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
                        
                        # Record the object information
                        self._record_found_object(pose_name)
                        
                        # Reset detection state to allow for continued scanning
                        self.scanner.reset_detection_state()
                        
                        object_found = True
                        break  # Exit the loop after successful approach
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
                
                if not approach_success:
                    rospy.logwarn(f"Failed to approach object after {max_attempts} attempts")
            else:
                rospy.loginfo("No object detected at final position")
            
            # Always return success=True if we reached the pose, even if approach failed
            return True, object_found
        else:
            return False, False
    
    def _record_found_object(self, pose_name):
        """
        Record information about a found object
        
        Args:
            pose_name: The pose name where the object was found
        """
        # Gather object data - modify based on what information is available
        # from your object_scanner and approacher
        object_info = {
            'location': pose_name,
            'timestamp': rospy.Time.now(),
            'marker': self.scanner.object_marker,  # Assuming this contains relevant object data
            'coordinates': {
                'x': self.approacher.final_position.x if hasattr(self.approacher, 'final_position') else 0,
                'y': self.approacher.final_position.y if hasattr(self.approacher, 'final_position') else 0
            }
        }
        
        self.found_objects.append(object_info)
        rospy.loginfo(f"Recorded object #{len(self.found_objects)} at {pose_name}")
        
    def scan_for_additional_objects(self, pose_name):
        """
        Scan for additional objects at the current position
        
        Args:
            pose_name: Current pose name
            
        Returns:
            int: Number of additional objects found
        """
        additional_objects = 0
        max_additional_scans = 3  # Limit the number of additional scans
        
        for i in range(max_additional_scans):
            # Rotate by 120 degrees to scan another sector
            offset_angle = 45 * (i + 1)
            rospy.loginfo(f"Performing additional scan with {offset_angle} degree rotation")
            
            # Reset detection state before additional scan
            self.scanner.reset_detection_state()
            
            # Perform the offset scan
            detection = self.scanner.resume_scan_with_offset(offset_degrees=offset_angle)
            
            if detection == ScanResult.OBJECT_DETECTED:
                # Try to approach the newly detected object
                approach_success = self.approacher.approach_object(self.scanner.object_marker)
                
                if approach_success:
                    rospy.loginfo(f"Successfully approached additional object at {pose_name}")
                    self._record_found_object(pose_name)
                    additional_objects += 1
                    
                    # Reset detection state for next scan
                    self.scanner.reset_detection_state()
                else:
                    rospy.logwarn(f"Failed to approach additional object")
            else:
                rospy.loginfo(f"No additional objects detected at {offset_angle} degrees")
                # If no object found, no need to continue additional scans
                break
                
        return additional_objects
        
    def return_to_pose(self, pose_name):
        """
        Return to the original pose after completing offset scans
        
        Args:
            pose_name: Name of the pose to return to
            
        Returns:
            bool: Success flag
        """
        target_pose = self.pose_manager.get_pose(pose_name)
        if target_pose is None:
            return False
            
        rospy.loginfo(f"Returning to original pose: {pose_name}")
        success = self.nav_controller.move_to_position(
            target_pose.position.x,
            target_pose.position.y
        )
        
        return success
        
    def move_to_all_poses(self):
        """
        Move through all poses, scanning and approaching all objects found
        
        Returns:
            int: Number of objects found
        """
        total_objects_found = 0
        
        for pose_name in self.pose_manager.get_all_pose_names():
            # Check if we've reached the maximum number of objects (if configured)
            if self.max_objects > 0 and total_objects_found >= self.max_objects:
                rospy.loginfo(f"Reached maximum number of objects ({self.max_objects}), ending navigation sequence")
                break
                
            rospy.loginfo(f"\nMoving to: {pose_name}")
            success, object_found = self.move_to_named_pose(pose_name)
            
            # If object found, increment counter and scan for additional objects
            if success and object_found:
                total_objects_found += 1
                rospy.loginfo(f"Object #{total_objects_found} detected and approached at {pose_name}")
                
                # Store original pose coordinates to return later
                original_pose = self.pose_manager.get_pose(pose_name)
                
                # Scan for additional objects at this position
                additional_objects = self.scan_for_additional_objects(pose_name)
                total_objects_found += additional_objects
                
                # Return to the original pose position to continue the sequence
                # This ensures we don't end up stuck in an offset position
                if additional_objects > 0:
                    self.return_to_pose(pose_name)
                
                # Check again if we've reached the maximum
                if self.max_objects > 0 and total_objects_found >= self.max_objects:
                    rospy.loginfo(f"Reached maximum number of objects ({self.max_objects}), ending navigation sequence")
                    break

            if not success:
                rospy.logwarn(f"Failed to reach {pose_name}, continuing to next pose...")
                
            rospy.sleep(1)  # Brief pause between movements
            
        return total_objects_found
        
    def main(self):
        """
        Main execution function
        
        Returns:
            int: Number of objects found
        """
        rospy.loginfo("=== Object Scout Mission Started ===")
        num_objects = self.move_to_all_poses()
        
        if num_objects > 0:
            rospy.loginfo(f"Successfully completed mission and found {num_objects} object(s)!")
            
            # Log information about all found objects
            for i, obj in enumerate(self.found_objects):
                rospy.loginfo(f"Object #{i+1} found at {obj['location']}")
        else:
            rospy.loginfo("Completed mission, no objects approached")
            
        return num_objects


if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinator(init_node=True)
        num_objects = coordinator.main()
        if num_objects == 0:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")