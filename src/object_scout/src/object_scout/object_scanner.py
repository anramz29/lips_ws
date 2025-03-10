#!/usr/bin/env python3
import rospy
import math
from enum import Enum
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32MultiArray
from object_scout.navigation_controller import NavigationController

# Import actionlib for navigation
import actionlib

# Import utilities
from object_scout.utils import get_robot_pose


class ScanResult(Enum):
    """Enumeration for scan operation results"""
    NO_DETECTION = 0
    OBJECT_DETECTED = 1
    ERROR = 2


class ObjectScanner:
    """
    Handles object detection through scanning rotations
    """
    def __init__(self, robot_name, nav_controller, init_node=False):
        """
        Initialize the scanner with robot name and navigation controller
        
        Args:
            robot_name: Name of the robot for topic namespacing
            nav_controller: Instance of NavigationController for rotation commands
            init_node: Whether to initialize a ROS node (standalone mode)
        """
        if init_node:
            rospy.init_node('object_scanner', anonymous=False)
            
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        
        # Scanner state
        self.scanning_in_progress = False
        self.object_marker = None
        self.object_detected = False
        self.current_depth = None
        
        # Scanner parameters
        self.scan_stabilization_time = 1.0  # seconds
        self.required_detection_duration = rospy.Duration(2.0)  # seconds
        self.scan_timeout = rospy.Duration(3.0)  # seconds
        
        # Setup marker subscription
        self.object_marker_topic = rospy.get_param('~object_marker_topic', 
                                                  f'/{robot_name}/detected_object/marker')
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )

                # Set up depth subscription
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 
                                               f'/{robot_name}/camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )

        self.rotation_angles =  [0, 45, 90, 135, 180, 225, 270, 315]

        self.remaining_angles = []
        
    def perform_scan_rotation(self, rotation_angles):
        """
        Perform a sequence of rotations to scan for objects
        
        The robot will rotate to several orientations, pausing at each to allow
        for stabilized object detection with a sustained detection requirement.
        
        Returns:
            ScanResult: The result of the scanning operation
        """
        rospy.loginfo("Starting scan rotation sequence")
        
        # Set scanning flag to true
        self.scanning_in_progress = True
        self.object_detected = False    
        self.remaining_angles = rotation_angles
        
        # Get current position
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return ScanResult.ERROR
    
        
        # Perform the rotations one by one
        for i in range(len(rotation_angles)):
            # Get the current angle to process (always the first in remaining list)
            if not self.remaining_angles:
                rospy.logwarn("No remaining angles to scan")
                break
                
            current_angle = self.remaining_angles[0]
            rospy.loginfo(f"Processing angle {i+1}/{len(rotation_angles)}: {current_angle} degrees")
            
            # Perform the scan at this angle
            result = self._perform_single_rotation_scan(current_pose, current_angle, i, len(rotation_angles))
            
            # Remove this angle from remaining angles AFTER processing it
            self.remaining_angles.pop(0)
            rospy.loginfo(f"Remaining angles after processing {current_angle} degrees: {self.remaining_angles}")
            
            # If object detected, return early with remaining angles
            if result == ScanResult.OBJECT_DETECTED:
                self.scanning_in_progress = False
                rospy.loginfo(f"Object detected during scan rotation at angle {current_angle}")
                return ScanResult.OBJECT_DETECTED, self.remaining_angles

            # Brief pause between rotations
            if i < len(rotation_angles) - 1:  # Don't pause after the last rotation
                rospy.sleep(0.5)
        
        # End of rotation sequence - no detection
        self.scanning_in_progress = False
        rospy.loginfo("Completed scan rotation sequence, no objects detected")
        return ScanResult.NO_DETECTION, self.remaining_angles
        
    def _perform_single_rotation_scan(self, current_pose, angle, rotation_index, total_rotations):
        """
        Perform a single rotation and scan for objects
        
        Args:
            current_pose: Current robot pose
            angle: Rotation angle in degrees
            rotation_index: Index of the current rotation
            total_rotations: Total number of rotations
            
        Returns:
            bool: True if object detected, False otherwise
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
        timeout = rospy.Duration(45.0)
        if not self.nav_controller.client.wait_for_result(timeout):
            rospy.logwarn("Rotation timeout reached")
            return False
        
        if self.nav_controller.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Rotation goal failed")
            return False
        
        # Rotation complete, wait for camera to stabilize
        rospy.loginfo(f"Rotation to {angle} degrees complete, waiting for camera stabilization...")
        rospy.sleep(self.scan_stabilization_time)
        
        # Clear any previous detection that might be from motion blur
        self.object_marker = None
        
        # Scan for objects with sustained detection requirement
        detected = self._scan_with_sustained_detection(angle)

        
        # Convert boolean result to ScanResult enum
        return ScanResult.OBJECT_DETECTED if detected else ScanResult.NO_DETECTION

        
    def _scan_with_sustained_detection(self, angle):
        """
        Scan for objects at current position with sustained detection requirement
        
        Args:
            angle: Current rotation angle for logging
            
        Returns:
            bool: True if object detected with sustained requirement, False otherwise
        """
        rospy.loginfo(f"Starting object detection at {angle} degrees...")
        
        # Variables for sustained detection
        detection_start = None
        scan_start = rospy.Time.now()
        
        while (rospy.Time.now() - scan_start) < self.scan_timeout and not rospy.is_shutdown():
            # Check if we have a marker
            if self.object_marker is not None:
                # Start or continue timing the sustained detection
                if detection_start is None:
                    detection_start = rospy.Time.now()
                    rospy.loginfo("Potential object detected, timing sustained detection...")
                else:
                    # Check if we've maintained detection long enough
                    elapsed = rospy.Time.now() - detection_start
                    if elapsed >= self.required_detection_duration:
                        if self.current_depth is not None and self.current_depth < 5.0:
                            
                            self.object_detected = True
                            
                            rospy.loginfo(f"Object detected at {angle} degrees with depth {self.current_depth}")
                            
                            return True
            else:
                # Lost detection, reset the timer
                if detection_start is not None:
                    rospy.logwarn("Lost detection during sustainment period, resetting timer")
                    detection_start = None
            
            rospy.sleep(0.1)  # Check at 10Hz
        
        return False  # No sustained detection within timeout

        
    def object_marker_callback(self, msg):
        """
        Process object marker messages
        
        Args:
            msg: Marker message from object detection
        """
        if self.scanning_in_progress:
            if msg.header.frame_id != "map":
                rospy.logwarn(f"Marker not in map frame: {msg.header.frame_id}")
                return
        
            # Store the validated marker
            self.object_marker = msg
            
    def reset_detection_state(self):
        """Reset object detection state"""
        self.object_detected = False
        self.object_marker = None

    def get_remaining_angles(self):
        return self.rotation_angles


    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg: Float32MultiArray containing depth data
        """
        if msg.data and len(msg.data) >= 5:
            self.current_depth = msg.data[4]
        else:
            rospy.logwarn("Invalid depth data received")

if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        # You would need to create a node that instantiates both
        rospy.loginfo("ObjectScanner cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object scanner interrupted")