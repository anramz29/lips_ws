#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
from object_scout.navigation_controller import NavigationController
from object_scout.object_scanner import ObjectScanner, ScanResult
from object_scout.object_approacher import ObjectApproacher
from object_scout.pose_manager import PoseManager
from object_scout.fine_approacher import FineApproacher
from object_scout.pick_up_object import PickUpObject
from object_scout.utils import get_robot_pose

# ---------- CLASS DEFINITION ----------

class ScoutCoordinatorLocobot:
    """
    Main coordinator for the object scouting mission
    
    This class orchestrates the overall scouting mission by:
    1. Navigating to predefined poses in the environment
    2. Scanning for objects at each pose with 360-degree rotation
    3. Approaching detected objects for closer inspection
    4. Fine-tuning the approach for optimal viewing
    5. Returning to scan positions to continue searching
    """
    def __init__(self, init_node=True):
        """
        Initialize the scouting coordinator with all required components
        
        Args:
            init_node (bool): Whether to initialize a ROS node
        """
        # ---------- ROS INITIALIZATION ----------
        
        if init_node:
            rospy.init_node('scout_coordinator', anonymous=False)
        
        # ---------- CONFIGURATION PARAMETERS ----------
        
        # Load parameters from ROS parameter server
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        self.max_objects = rospy.get_param('~max_objects', 0)  # 0 means find all objects
        
        # Frame definitions
        self.world_frame = "map"
        self.robot_frame = f"{self.robot_name}/base_link"
        
        # ---------- COMPONENT INITIALIZATION ----------
        
        # Setup TF listener for coordinate transforms
        self._initialize_tf_listener()
        
        # Initialize all required components
        self._initialize_components()
        
        # ---------- STATE VARIABLES ----------
        
        # Track found objects
        self.found_objects = []

    def _initialize_tf_listener(self):
        """Initialize the TF listener for coordinate transformations"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _initialize_components(self):
        """Initialize all required components for the scouting mission"""

        
        # Create pose manager for handling predefined positions
        self.pose_manager = PoseManager(self.poses_config)

               # Create navigation controller
        self.nav_controller = NavigationController(self.robot_name, self.pose_manager)
        
        # Create scanner for detecting objects
        self.scanner = ObjectScanner(self.robot_name, self.nav_controller)
        
        # Create approacher for moving to detected objects
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller)
        
        # Create fine approacher for precise positioning
        self.fine_approacher = FineApproacher(self.robot_name, self.nav_controller)

        # Create object picker for picking up objects
        self.object_picker = PickUpObject(self.robot_name)

    # ---------- OBJECT DETECTION METHODS ----------

    def scan_for_objects(self, angles=None):
        """
        Perform a rotation scan for objects
        
        Args:
            angles (list, optional): List of angles to scan, or None for default
            
        Returns:
            tuple: (scan_result, remaining_angles)
        """
        rospy.loginfo("Scanning for objects...")
        scan_angles = angles if angles is not None else self.scanner.rotation_angles
        return self.scanner.perform_scan_rotation(scan_angles)

    def approach_detected_object(self):
        """
        Approach a detected object
        
        Returns:
            bool: True if approach succeeded, False otherwise
        """
        rospy.loginfo("Approaching detected object...")
        object_marker = self.scanner.object_marker
        
        if object_marker is None:
            rospy.logwarn("No object marker available for approach")
            return False
            
        return self.approacher.approach_object(object_marker)

    def perform_fine_approach(self):
        """
        Perform a fine approach to center the object in view
        
        Returns:
            bool: True if fine approach succeeded, False otherwise
        """
        rospy.loginfo("Performing fine approach...")
        return self.fine_approacher.fine_approach()

    # ---------- MISSION EXECUTION METHODS ----------

    def process_detected_object(self, pose_name, x_detected, y_detected, orientation_detected, remaining_angles):
        """
        Process a detected object - approach, fine-tune, and return to scan position
        
        Args:
            pose_name (str): Name of the pose where object was detected
            x_detected (float): X coordinate where object was detected
            y_detected (float): Y coordinate where object was detected
            orientation_detected: Orientation where object was detected
            remaining_angles (list): Remaining scan angles
            
        Returns:
            tuple: (scan_result, remaining_angles) for next scan
        """
        # Approach the detected object
        approach_success = self.approach_detected_object()

        if approach_success:
            rospy.loginfo("Successfully approached object")

            # Perform fine approach
            fine_approach_success = self.perform_fine_approach()

            if fine_approach_success:
                # Log success and update object count
                rospy.loginfo(f"Successfully completed approach to object at pose {pose_name}")
                rospy.sleep(1.0)  # Pause for a moment
                self.found_objects.append(pose_name)

                # pick up object 
                if self.object_picker.run():
                    rospy.loginfo("Object picked up")
                else:
                    rospy.logwarn("Object not picked up")

                # Reset camera tilt angle
                self.fine_approacher.reset_camera_tilt()
            else:
                rospy.logwarn("Fine approach failed")
        else:
            rospy.logwarn("Failed to approach object")

        # Return to the position where the object was detected
        self.nav_controller.return_to_position(x_detected, y_detected, orientation_detected)

        # Perform another scan rotation with remaining angles
        return self.scan_for_objects(remaining_angles)

    def check_mission_complete(self):
        """
        Check if the mission is complete based on max objects found
        
        Returns:
            bool: True if mission complete, False otherwise
        """
        if self.max_objects > 0 and len(self.found_objects) >= self.max_objects:
            rospy.loginfo(f"Reached maximum number of objects ({self.max_objects}), mission complete")
            return True
        return False

    def execute_scouting_mission(self):
        """
        Execute the complete object scouting mission
        
        This method orchestrates the entire scouting process:
        1. Reset camera to default position
        2. Navigate to each predefined pose
        3. Scan for objects with 360-degree rotation
        4. Approach and document any detected objects
        5. Continue until all poses are visited or max objects found
        
        Returns:
            int: Number of objects found during the mission
        """
        # Reset camera tilt angle to start
        self.fine_approacher.reset_camera_tilt()

        # Get all poses to visit
        rospy.loginfo("Starting object scouting mission")
        pose_names = self.pose_manager.get_all_pose_names()

        # Visit each pose sequentially
        for pose_name in pose_names:
            # Navigate to the pose
            if not self.nav_controller.navigate_to_named_pose(pose_name):
                continue  # Skip this pose if navigation failed

            # Check if we've already found enough objects
            if self.check_mission_complete():
                break

            # Initial scan at the pose
            scan_result, remaining_angles = self.scan_for_objects()
            
            # Process any detected objects
            while scan_result == ScanResult.OBJECT_DETECTED:
                # Get position where object was detected
                x_detected, y_detected, orientation_detected = self.nav_controller.get_robot_pose()
                
                # Process the detected object
                scan_result, remaining_angles = self.process_detected_object(
                    pose_name, x_detected, y_detected, orientation_detected, remaining_angles
                )
                
                # Check if we've reached the maximum number of objects
                if self.check_mission_complete():
                    break
                    
                # If no more objects detected, exit the inner loop
                if scan_result != ScanResult.OBJECT_DETECTED:
                    rospy.loginfo("No more objects detected in current scan position")
                    break
                    
        # Mission complete
        rospy.loginfo(f"Mission complete. Found {len(self.found_objects)} objects: {self.found_objects}")
        return len(self.found_objects)

# ---------- DIRECT EXECUTION BLOCK ----------

if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinatorLocobot(init_node=True)
        num_objects = coordinator.execute_scouting_mission()
        if num_objects == 0:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")