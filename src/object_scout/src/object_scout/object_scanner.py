#!/usr/bin/env python3
import rospy
import math
from enum import Enum
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray

# Import actionlib for navigation
import actionlib


# ---------- ENUMERATIONS ----------

class ScanResult(Enum):
    """Enumeration for scan operation results"""
    NO_DETECTION = 0
    OBJECT_DETECTED = 1
    ERROR = 2

# ---------- CLASS DEFINITION ----------

class ObjectScanner:
    """
    Handles object detection through scanning rotations
    
    This class provides functionality to:
    1. Rotate the robot through a sequence of angles
    2. Detect objects at each rotation angle
    3. Verify detections with sustained observation
    4. Track scanning progress and remaining angles
    """
    def __init__(self, robot_name, nav_controller, object_mapper_topic,
                 bbox_depth_topic, init_node=False):
        """
        Initialize the scanner with robot name and navigation controller
        
        Args:
            robot_name (str): Name of the robot for topic namespacing
            nav_controller: Instance of NavigationController for rotation commands
            init_node (bool): Whether to initialize a ROS node (standalone mode)
        """
        # Initialize node if requested
        if init_node:
            rospy.init_node('object_scanner', anonymous=False)
        
        # ---------- CONFIGURATION ----------
            
        # Core components
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        self.object_mapper_topic = object_mapper_topic
        self.bbox_depth_topic = bbox_depth_topic
        
        # Scanner parameters
        self.scan_stabilization_time = 1.0  # seconds
        self.required_detection_duration = rospy.Duration(1.0)  # seconds
        self.scan_timeout = rospy.Duration(1.5)  # seconds
        self.rotation_timeout = rospy.Duration(45.0)  # seconds
        self.max_detection_depth = 10.0  # meters
        
        # Default rotation angles in degrees
        self.rotation_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        
        # ---------- STATE VARIABLES ----------
        
        # Scanner state
        self.scanning_in_progress = False
        self.object_marker = None
        self.object_detected = False
        self.current_depth = None
        self.current_class_id = None
        self.remaining_angles = []
        
        # ---------- ROS INTERFACE SETUP ----------
        
        # Set up marker subscription
        self._setup_marker_subscription()
        
        # Set up depth subscription
        self._setup_depth_subscription()
    
    # ---------- ROS COMMUNICATION SETUP ----------
    
    def _setup_marker_subscription(self):
        """Set up ROS subscription for object marker information"""

        
        self.object_marker_sub = rospy.Subscriber(
            self.object_mapper_topic,
            Marker,
            self.object_marker_callback
        )
    
    def _setup_depth_subscription(self):
        """Set up ROS subscription for depth information"""
        
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )
    
    # ---------- CALLBACK METHODS ----------
    
    def object_marker_callback(self, msg):
        """
        Process object marker messages
        
        Args:
            msg (Marker): Marker message from object detection
        """
        if self.scanning_in_progress:
            if msg.header.frame_id != "map":
                rospy.logwarn(f"Marker not in map frame: {msg.header.frame_id}")
                return
        
            # Store the validated marker
            self.object_marker = msg
    
    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg (Float32MultiArray): Message containing depth data
                Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
        """
        data_length = len(msg.data)

        # Check for minimum data length required by new format
        if data_length >= 8:
            n_boxes = int(msg.data[0])
            if n_boxes < 1:
                rospy.logdebug("No boxes detected in depth message")
                return
             # Extract depth from the new format - position 7
            self.current_depth = float(msg.data[7])

            # Extract class ID from the new format - position 1
            self.current_class_id = int(msg.data[1])
                
    
    # ---------- STATE MANAGEMENT METHODS ----------
    
    def reset_detection_state(self):
        """Reset object detection state"""
        self.object_detected = False
        self.object_marker = None
    
    def get_remaining_angles(self):
        """
        Get the list of remaining scan angles
        
        Returns:
            list: Remaining angles to scan in degrees
        """
        return self.remaining_angles
    
    # ---------- DETECTION METHODS ----------
    
    def _scan_with_sustained_detection(self, angle, desired_class_id=None):
        """
        Scan for objects at current position with sustained detection requirement
        
        Requires that an object is continuously detected for a minimum duration
        to filter out false positives and ensure stable detection.
        
        Args:
            angle (float): Current rotation angle for logging
            desired_class_id (int, optional): Class ID to scan for, or None to accept any class
            
        Returns:
            bool: True if object detected with sustained requirement, False otherwise
        """
        rospy.loginfo(f"Starting object detection at {angle} degrees...")
        
        # Variables for sustained detection
        detection_start = None
        scan_start = rospy.Time.now()
        
        # Continue scanning until timeout
        while (rospy.Time.now() - scan_start) < self.scan_timeout and not rospy.is_shutdown():
                
            # Check if we have a marker
            if self.object_marker is not None:
                # Accept any class ID if desired_class_id is None, otherwise check for match
                class_match = (desired_class_id is None) or (self.current_class_id == desired_class_id)
                
                if class_match:
                    rospy.loginfo_throttle(5.0, f"Detected object with class ID {self.current_class_id}")

                    # Start or continue timing the sustained detection
                    if detection_start is None:
                        detection_start = rospy.Time.now()
                        rospy.loginfo("Potential object detected, timing sustained detection...")
                    else:
                        # Check if we've maintained detection long enough
                        elapsed = rospy.Time.now() - detection_start
                        if elapsed >= self.required_detection_duration:
                            # Validate detection with depth check
                            if self.current_depth is not None and self.current_depth < self.max_detection_depth:
                                self.object_detected = True
                                rospy.loginfo(f"Object detected at {angle} with class id {self.current_class_id}")
                                return True          
                else:
                    # Detected object does not match desired class, reset timer
                    rospy.logwarn_throttle(2.0, f"Detected object with incorrect class ID {self.current_class_id}")
                    detection_start = None
            else:
                # Lost detection, reset the timer
                if detection_start is not None:
                    rospy.logwarn("Lost detection during sustainment period, resetting timer")
                    detection_start = None      
            rospy.sleep(0.1)  # Check at 10Hz
        
        rospy.loginfo(f"No sustained detection at {angle} degrees within timeout")
        return False
    
    # ---------- ROTATION METHODS ----------
    
    def _perform_single_rotation_scan(self, current_pose, angle, rotation_index, total_rotations, desired_class_id):
        """
        Perform a single rotation and scan for objects
        
        1. Rotates to the specified angle
        2. Waits for camera stabilization
        3. Scans for objects with sustained detection requirement
        
        Args:
            current_pose: Current robot pose
            angle (float): Rotation angle in degrees
            rotation_index (int): Index of the current rotation
            total_rotations (int): Total number of rotations
            
        Returns:
            ScanResult: Result of the scan operation
        """
        # Reset object detection status for this rotation
        self.object_marker = None
        self.object_detected = False
        
        # Calculate orientation quaternion
        yaw = math.radians(angle)
        quaternion = quaternion_from_euler(0, 0, yaw)
        
        # Create goal at current position with new orientation
        current_goal = self.nav_controller.create_navigation_goal(
            current_pose.position.x,
            current_pose.position.y,
            Quaternion(*quaternion)
        )
        
        # Send rotation goal
        rospy.loginfo(f"Executing rotation {rotation_index+1}/{total_rotations} ({angle} degrees)")
        self.nav_controller.client.send_goal(current_goal)
        
        # Wait for rotation to complete with timeout
        if not self.nav_controller.client.wait_for_result(self.rotation_timeout):
            rospy.logwarn(f"Rotation timeout reached for angle {angle}")
            return ScanResult.ERROR
        
        if self.nav_controller.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn(f"Rotation goal failed for angle {angle}")
            return ScanResult.ERROR
        
        # Rotation complete, wait for camera to stabilize
        rospy.loginfo(f"Rotation to {angle} degrees complete, waiting {self.scan_stabilization_time}s for stabilization...")
        rospy.sleep(self.scan_stabilization_time)
        
        # Clear any previous detection that might be from motion blur
        self.object_marker = None
        
        # Scan for objects with sustained detection requirement
        detected = self._scan_with_sustained_detection(angle, desired_class_id)
        
        # Convert boolean result to ScanResult enum
        return ScanResult.OBJECT_DETECTED if detected else ScanResult.NO_DETECTION
    
    # ---------- MAIN SCANNING METHOD ----------
    
    def perform_scan_rotation(self, rotation_angles=None, desired_class_id=None, complete_scan=False):
        """
        Perform a sequence of rotations to scan for objects
        
        The robot will rotate to several orientations, pausing at each to allow
        for stabilized object detection with a sustained detection requirement.
        
        Args:
            rotation_angles (list, optional): List of angles in degrees to scan.
                                             If None, uses default angles.
            desired_class_id (int, optional): Class ID to scan for.
            complete_scan (bool, optional): If True, completes all rotations regardless
                                           of detections. If False, exits early on first detection.
        
        Returns:
            tuple: (ScanResult, remaining_angles) where:
                  - ScanResult is the result of the scanning operation
                  - remaining_angles is a list of angles not yet scanned
        """
        if rotation_angles is None:
            rotation_angles = self.rotation_angles.copy()
        
        rospy.loginfo(f"Starting scan rotation sequence with {len(rotation_angles)} angles: {rotation_angles}")
        rospy.loginfo(f"Scan mode: {'Complete scan' if complete_scan else 'Early exit on detection'}")
        
        # Set scanning flag to true and initialize state
        self.scanning_in_progress = True
        self.object_detected = False    
        self.remaining_angles = rotation_angles.copy()
        found_object = False
        
        # Get current position
        current_pose = self.nav_controller.get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return ScanResult.ERROR, self.remaining_angles
        
        # Perform the rotations one by one
        for i in range(len(rotation_angles)):
            # Get the current angle to process (always the first in remaining list)
            if not self.remaining_angles:
                rospy.logwarn("No remaining angles to scan")
                break
                
            current_angle = self.remaining_angles[0]
            rospy.loginfo(f"Processing angle {i+1}/{len(rotation_angles)}: {current_angle} degrees")
            
            # Perform the scan at this angle
            result = self._perform_single_rotation_scan(
                current_pose, current_angle, i, len(rotation_angles), desired_class_id
            )
            
            # Remove this angle from remaining angles AFTER processing it
            self.remaining_angles.pop(0)
            rospy.loginfo(f"Remaining angles after processing {current_angle} degrees: {self.remaining_angles}")
            
            # If object detected, either return early or continue based on complete_scan flag
            if result == ScanResult.OBJECT_DETECTED:
                found_object = True
                rospy.loginfo(f"Object detected during scan rotation at angle {current_angle}")
                if not complete_scan:
                    self.scanning_in_progress = False
                    return ScanResult.OBJECT_DETECTED, self.remaining_angles
                # If complete_scan is True, continue to next angle
            
            # If error occurred, log and continue to next angle
            if result == ScanResult.ERROR:
                rospy.logwarn(f"Error during scan at angle {current_angle}, continuing to next angle")
            
            # Brief pause between rotations
            if i < len(rotation_angles) - 1:  # Don't pause after the last rotation
                rospy.sleep(0.5)
        
        # End of rotation sequence
        self.scanning_in_progress = False
        
        if found_object:
            rospy.loginfo("Completed scan rotation sequence, detected objects at one or more angles")
            return ScanResult.OBJECT_DETECTED, self.remaining_angles
        else:
            rospy.loginfo("Completed scan rotation sequence, no objects detected")
            return ScanResult.NO_DETECTION, self.remaining_angles


# ---------- DIRECT EXECUTION BLOCK ----------

if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        rospy.loginfo("ObjectScanner cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object scanner interrupted")