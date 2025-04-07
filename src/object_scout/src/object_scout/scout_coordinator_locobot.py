#!/usr/bin/env python3
import rospy
import tf2_ros
from object_scout.navigation_controller import NavigationController
from object_scout.object_scanner import ObjectScanner, ScanResult
from object_scout.object_approacher import ObjectApproacher
from object_scout.pose_manager import PoseManager
from object_scout.fine_approacher import FineApproacher
from object_scout.pick_up_object import PickUpObject
from object_scout.place_object import PlaceObject

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
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.depth_topic = rospy.get_param('~depth_topic')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        self.object_marker_topic = rospy.get_param('~object_marker_topic')
        self.costmap_topic = rospy.get_param('~costmap_topic')
        self.move_base_topic = rospy.get_param('~move_base_topic')
        self.move_base_cancel_topic = rospy.get_param('~move_base_cancel_topic')
        self.camera_joint_topic = rospy.get_param('~camera_joint_topic')
        self.keypoint_angle_topic = rospy.get_param('~keypoint_angle_topic')
        self.enable_keypoint_detection_service = rospy.get_param('~enable_keypoint_detection_service')



        
        # Frame definitions
        self.world_frame = "map"
        self.robot_frame = f"{self.robot_name}/base_link"
        
        # ---------- COMPONENT INITIALIZATION ----------

        # Initialize variables
        self.detected_object_poses = []
        self.found_objects = []
        
        # Setup TF listener for coordinate transforms
        self._initialize_tf_listener()
        
        # Initialize all required components
        self._initialize_components()
        
        # Register shutdown handler
        rospy.on_shutdown(self.shutdown_handler)

    def _initialize_tf_listener(self):
        """Initialize the TF listener for coordinate transformations"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _initialize_components(self):
        """Initialize all required components for the scouting mission"""

        # Create pose manager for handling predefined positions
        self.pose_manager = PoseManager(self.poses_config)

               # Create navigation controller
        self.nav_controller = NavigationController(
            self.robot_name, 
            self.pose_manager,
            self.move_base_topic, 
            self.costmap_topic,
            self.move_base_cancel_topic
        )
        
        # Create scanner for detecting objects
        self.scanner = ObjectScanner(
            self.robot_name, 
            self.nav_controller,
            self.object_marker_topic,
            self.bbox_depth_topic  
        )
        
        # Create approacher for moving to detected objects
        self.approacher = ObjectApproacher(
            self.robot_name, 
            self.nav_controller,
            self.object_marker_topic,
            self.bbox_depth_topic
        )
        
        # Create fine approacher for precise positioning
        self.fine_approacher = FineApproacher(
            self.robot_name, 
            self.nav_controller,
            self.bbox_depth_topic, 
            self.camera_joint_topic
        )

        # Create object picker for picking up objects
        self.object_picker = PickUpObject(
            self.robot_name,
            self.fine_approacher,
            self.keypoint_angle_topic,
            self.enable_keypoint_detection_service
        )

        self.object_placer = PlaceObject(
            self.robot_name,
            self.bbox_depth_topic,
            self.camera_joint_topic,
            self.depth_topic
        )

    # ---------- OBJECT DETECTION METHODS ----------

    def intial_scan_rotation(self):
        """Perform the initial obtain data of all detected objects in the room"""

        rospy.loginfo("Starting initial scan...")
        scan_result = self.scanner.perform_scan_rotation(complete_scan=False, desired_class_id=0)
        if scan_result:
            rospy.loginfo("Initial scan complete")
            return True
        else:
            rospy.logwarn("No objects detected during initial scan.")

    def approach_and_save_box(self):
        """Approach the detected box and save its pose for later use"""
        # Approach the detected object
        self.approacher.approach_object(approach_max_depth=.8, approach_min_depth=.5)
        position = self.nav_controller.get_robot_pose()
        if position:
            rospy.loginfo(f"Box pose saved: {position}")
            return position
        else:
            rospy.logwarn("Failed to save box pose.")
            return None

    def approach_and_pick_up_object(self, pose_name):
        """Approach the detected object and pick it up"""
        
        approach_success = self.approacher.approach_object()
        if approach_success:
            rospy.loginfo("Successfully approached the object.")
            # Perform fine approach
            fine_approach_success = self.fine_approacher.fine_approach(desired_vertical=0.8)

            if fine_approach_success:
                # Log success and update object count
                rospy.loginfo(f"Successfully completed approach fine approach.")
                rospy.sleep(1.0)  # Pause for a moment
                if self.object_picker.pick_object_with_retries():
                    rospy.loginfo("Object picked up")
                    # Reset camera tilt angle
                    self.fine_approacher.reset_camera_tilt()

                    # add the picked object to the found objects list
                    self.found_objects.append(pose_name)
                    return True
                else:
                    rospy.logwarn("Object not picked up")
                    # Reset camera tilt angle
                    self.fine_approacher.reset_camera_tilt()
                    return False
            else:
                rospy.logwarn("Fine approach failed")
                # Reset camera tilt angle
                self.fine_approacher.reset_camera_tilt()
                return False
        else:
            rospy.logwarn("Failed to approach object")
            return False
        
    def place_object(self, box_pose):
        """Place the picked object at a designated location"""
    
        # Access position values correctly through the position attribute
        if self.nav_controller.move_to_position(
                box_pose.position.x,  # Use position.x instead of x
                box_pose.position.y,  # Use position.y instead of y
                box_pose.orientation  # Pass orientation instead of z
            ):
            self.nav_controller.clear_costmaps()
            rospy.loginfo("Approaching object placement position...")
            if self.fine_approacher.fine_approach(desired_vertical=0.6):
                rospy.loginfo("Successfully approached object placement position")
                if self.object_placer.place_at_bbox_center():
                    rospy.loginfo("Object placed successfully")
                    return True
                else:
                    rospy.logwarn("Failed to place object")
                    return False
            else:
                rospy.logwarn("Failed to fine approach object placement position")
                return False   
        else:
            rospy.logwarn("Failed to move to intermediate pose")
            return False

        
    # ---------- SCOUTING MISSION EXECUTION ----------

    def execute_scouting_mission(self):
        """
        Execute the complete object scouting mission
        
        Returns:
            tuple: (success, num_objects) - Whether mission was successful and number of objects found
        """
        # Reset camera tilt angle to start
        self.fine_approacher.reset_camera_tilt()

        self.nav_controller.clear_costmaps()
        
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

            if self.intial_scan_rotation():
                # Approach and save the box pose if detected
                box_pose = self.approach_and_save_box()
                if box_pose:
                    rospy.loginfo(f"Box pose saved: {box_pose}")
                else:
                    rospy.logwarn("No box detected or failed to save pose")
                    return False, 0
                
            if not self.nav_controller.navigate_to_named_pose(pose_name):
                continue    

            # Initial scan at the pose
            scan_result, remaining_angles = self.scanner.perform_scan_rotation(desired_class_id=1)
            
            # Process any detected objects
            while scan_result == ScanResult.OBJECT_DETECTED:
                # Get position where object was detected
                x_detected, y_detected, orientation_detected = self.nav_controller.get_robot_pose_postions()
                
                pick_up_success = self.approach_and_pick_up_object(pose_name)
                if pick_up_success:
                    # clear costmaps
                    self.nav_controller.clear_costmaps()
                    rospy.loginfo("Object picked up successfully")
                    # Place the object at a designated location
                    if self.place_object(box_pose):
                        rospy.loginfo("Object placed successfully")
                        # Return to the position where the object was detected
                        self.nav_controller.return_to_position(x_detected, y_detected, orientation_detected)
                        
                        # Perform another scan rotation to find more objects
                        scan_result, remaining_angles = self.scanner.perform_scan_rotation(remaining_angles, desired_class_id=1)
                        
                        if self.check_mission_complete():
                            rospy.loginfo("Reached maximum number of objects, mission complete")
                            break
                    else:
                        rospy.logwarn("Failed to place object")
                        return False, 0
                else:
                    rospy.logwarn("Failed to pick up object")
                    # Return to the position where the object was detected to try another scan
                    self.nav_controller.return_to_position(x_detected, y_detected, orientation_detected)
                    
                    # Perform another scan rotation with remaining angles
                    scan_result, remaining_angles = self.scanner.perform_scan_rotation(remaining_angles, desired_class_id=1)
                    continue
                
                # If no more objects detected, exit the inner loop
                if scan_result != ScanResult.OBJECT_DETECTED:
                    rospy.loginfo("No more objects detected in current scan position")
                    break
                    
        # Mission complete
        found_count = len(self.found_objects)
        rospy.loginfo(f"Mission complete. Found {found_count} objects: {self.found_objects}")
        return True, found_count
    

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
    

    def shutdown_handler(self):
        """Handle shutdown cleanup for the coordinator"""
        rospy.loginfo("Scout coordinator shutting down...")
        try:
            # Make sure the object picker shuts down cleanly
            if hasattr(self, 'object_picker'):
                self.object_picker.shutdown_handler()
                self.fine_approacher.shutdown_handler()
        except Exception as e:
            rospy.logerr(f"Error during coordinator shutdown: {e}")

# ---------- DIRECT EXECUTION BLOCK ----------

if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinatorLocobot(init_node=True)
        success, num_objects = coordinator.execute_scouting_mission()
        if not success or num_objects == 0:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")