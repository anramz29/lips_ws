#!/usr/bin/env python3
import rospy
import math
from enum import Enum
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

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
        
    def perform_scan_rotation(self):
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
        
        # Get current position
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return ScanResult.ERROR
        
        
        # Rotation angles - seven rotations of 45 degrees each
        rotation_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    
        num_rotations = len(rotation_angles)

        
        
        # Perform the rotations
        for i, angle in enumerate(rotation_angles):
            if self._perform_single_rotation_scan(current_pose, angle, i, num_rotations):
                # Object detected with sustained requirement
                return ScanResult.OBJECT_DETECTED
                
            # Brief pause between rotations
            if i < len(rotation_angles) - 1:  # Don't pause after the last rotation
                rospy.sleep(0.5)
        
        # End of rotation sequence - no detection
        self.scanning_in_progress = False
        rospy.loginfo("Completed scan rotation sequence, no objects detected")
        return ScanResult.NO_DETECTION
        
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
        return self._scan_with_sustained_detection(angle)
        
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
                        self.object_detected = True
                        rospy.loginfo(f"Sustained object detection for {elapsed.to_sec():.2f} seconds!")
                        return True
            else:
                # Lost detection, reset the timer
                if detection_start is not None:
                    rospy.logwarn("Lost detection during sustainment period, resetting timer")
                    detection_start = None
            
            rospy.sleep(0.1)  # Check at 10Hz
        
        return False  # No sustained detection within timeout
    
    def resume_scan_with_offset(self, offset_degrees=45):
        """
        Resume scanning from current position with an additional rotation offset
        
        This function:
        1. Determines the current rotation index
        2. Applies an additional rotation (default 45 degrees)
        3. Continues the scan with only the remaining rotations
        
        Args:
            offset_degrees: Additional degrees to rotate (default: 45)
            
        Returns:
            ScanResult: The result of the scanning operation
        """
        # First, save current position to determine where we are
        current_index = self.save_current_scan_position()
        if current_index == -1:
            rospy.logerr("Failed to determine current scan position")
            return ScanResult.ERROR
            
        # Get current position for the new rotation
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return ScanResult.ERROR
            
        # Calculate the new angle (current + offset)
        rotation_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        current_angle = rotation_angles[current_index]
        new_angle = (current_angle + offset_degrees) % 360
        
        rospy.loginfo(f"Current angle: {current_angle} degrees")
        rospy.loginfo(f"Adding offset of {offset_degrees} degrees")
        rospy.loginfo(f"New angle: {new_angle} degrees")
        
        # First, rotate to the offset position
        # Calculate orientation quaternion for the offset
        yaw = math.radians(new_angle)
        quaternion = quaternion_from_euler(0, 0, yaw)
        
        # Create goal at current position with new orientation
        offset_goal = self.nav_controller.create_navigation_goal(
            current_pose.position.x,
            current_pose.position.y,
            Quaternion(*quaternion)
        )
        
        # Send rotation goal for the offset
        rospy.loginfo(f"Executing offset rotation to {new_angle} degrees")
        self.nav_controller.client.send_goal(offset_goal)
        
        # Wait for rotation to complete with timeout
        timeout = rospy.Duration(45.0)
        if not self.nav_controller.client.wait_for_result(timeout):
            rospy.logwarn("Offset rotation timeout reached")
            return ScanResult.ERROR
        
        if self.nav_controller.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Offset rotation goal failed")
            return ScanResult.ERROR
        
        # Rotation complete, wait for camera to stabilize
        rospy.loginfo(f"Offset rotation to {new_angle} degrees complete, waiting for camera stabilization...")
        rospy.sleep(self.scan_stabilization_time)
        
        # Find the closest standard angle to our new position
        closest_angle_index = min(range(len(rotation_angles)), 
                                key=lambda i: abs(rotation_angles[i] - new_angle))
        
        # Now continue with the remaining rotations
        return self.resume_scan_from_saved_position(closest_angle_index)
    
    def resume_scan_from_saved_position(self, saved_angle_index):
        """
        Perform a scan rotation sequence starting from a saved position
        
        The robot will resume scanning from the last saved position,
        continuing with only the remaining rotations in the sequence.
        
        Args:
            saved_angle_index: Index of the last completed rotation angle
            
        Returns:
            ScanResult: The result of the scanning operation
        """
        rospy.loginfo(f"Resuming scan rotation sequence from index {saved_angle_index}")
        
        # Set scanning flag to true
        self.scanning_in_progress = True
        self.object_detected = False
        
        # Get current position
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return ScanResult.ERROR
        
        # Rotation angles - seven rotations of 45 degrees each
        rotation_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        
        # Calculate remaining rotations (starting from the next one)
        remaining_angles = rotation_angles[saved_angle_index + 1:]
        
        num_rotations = len(remaining_angles)
        if num_rotations == 0:
            rospy.loginfo("No remaining rotations to perform")
            self.scanning_in_progress = False
            return ScanResult.NO_DETECTION
        
        rospy.loginfo(f"Performing {num_rotations} remaining rotations")
        
        # Perform the remaining rotations
        for i, angle in enumerate(remaining_angles):
            if self._perform_single_rotation_scan(current_pose, angle, i, num_rotations):
                # Object detected with sustained requirement
                return ScanResult.OBJECT_DETECTED
                
            # Brief pause between rotations
            if i < len(remaining_angles) - 1:  # Don't pause after the last rotation
                rospy.sleep(0.5)
        
        # End of rotation sequence - no detection
        self.scanning_in_progress = False
        rospy.loginfo("Completed resumed scan rotation sequence, no objects detected")
        return ScanResult.NO_DETECTION


    def save_current_scan_position(self):
        """
        Save the current scan position for later resumption
        
        Returns:
            int: Index of the current rotation in the sequence, 
                or -1 if not currently scanning or position couldn't be determined
        """
        if not self.scanning_in_progress:
            rospy.logwarn("Cannot save position: No scan in progress")
            return -1
        
        # Get current position and orientation
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose to save position")
            return -1
        
        # Convert current orientation to degrees
        current_quat = [
            current_pose.orientation.x,
            current_pose.orientation.y, 
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        
        # Import here to avoid circular imports
        from tf.transformations import euler_from_quaternion
        
        roll, pitch, yaw = euler_from_quaternion(current_quat)
        current_degrees = math.degrees(yaw) % 360
        
        # Find the closest standard rotation angle
        rotation_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        closest_angle_index = min(range(len(rotation_angles)), 
                                key=lambda i: abs(rotation_angles[i] - current_degrees))
        
        rospy.loginfo(f"Current orientation: {current_degrees:.1f} degrees, closest to index {closest_angle_index} ({rotation_angles[closest_angle_index]} degrees)")
        
        # Save this position (you might want to expand this to save the actual pose too)
        self.saved_scan_index = closest_angle_index
        rospy.loginfo(f"Saved scan position at index {self.saved_scan_index}")
        
        return closest_angle_index
        
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


if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        # You would need to create a node that instantiates both
        rospy.loginfo("ObjectScanner cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object scanner interrupted")