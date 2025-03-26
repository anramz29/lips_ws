#!/usr/bin/env python3
import rospy
import math
import actionlib
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from interbotix_xs_msgs.msg import JointGroupCommand # type: ignore
import sensor_msgs.msg


class FineApproacher():
    """
    Class for fine approach of objects detected by a camera
    
    This class provides functionality to precisely approach objects by:
    1. Centering objects in the camera view both horizontally and vertically
    2. Making incremental base adjustments using visual feedback
    """
    def __init__(self, robot_name, nav_controller, init_node=False):
        """
        Initialize the FineApproacher with robot configuration and ROS setup
        
        Args:
            robot_name (str): Name of the robot
            nav_controller (NavigationController): Controller for robot navigation
            init_node (bool): Whether to initialize a ROS node
        """
        if init_node:
            rospy.init_node('fine_approach', anonymous=False)

        # Configuration parameters
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        self.navigation_timeout = 60.0
        self.default_camera_tilt = 0.2618
        self.image_height = 480
        self.image_width = 640
        self.center_threshold = 10.0
        
        # Centering parameters
        self.horizontal_tolerance = 20
        self.vertical_tolerance = 70
        self.horizontal_threshold = 100
        self.vertical_threshold = 70
        self.max_vertical_distance = 0.5  # meters
        self.max_rotation = 0.5  # radians, about 28.6 degrees
        self.min_rotation_threshold = 0.02  # radians
        self.min_movement_threshold = 0.015  # meters
        self.horizontal_damping = 1.0
        self.vertical_damping = 1.3

        # Bounding box and depth tracking
        self.bbox_depth = None
        self.y_min, self.x_min, self.y_max, self.x_max = 0, 0, 0, 0

        # Set up depth subscription
        self._setup_ros_communication()

    # ---------- ROS Communication Setup ----------

    def _setup_ros_communication(self):
        """Set up ROS publishers and subscribers"""
        # Set up depth subscription
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 
                                               f'/{self.robot_name}/camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.bbox_callback
        )

        # Create publisher for camera joint group
        self.camera_pub = rospy.Publisher(
            f'/{self.robot_name}/commands/joint_group', 
            JointGroupCommand, 
            queue_size=1
        )
    def bbox_callback(self, msg):
        """
        Callback function for bounding box messages
        
        Args:
            msg (Float32MultiArray): Message containing bounding box coordinates and depth
                Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
        """
        # if not msg.data or len(msg.data) < 8:
        #     rospy.logwarn("Received empty or invalid message")
        #     return
            
        # Extract values from new format
        n_boxes = int(msg.data[0])
        if n_boxes < 1:
            return
            
        # Get coordinates from the first detected box
        x1 = float(msg.data[3])
        y1 = float(msg.data[4])
        x2 = float(msg.data[5])
        y2 = float(msg.data[6])
        
        # Store the bounding box corners directly
        self.x_min = x1
        self.y_min = y1
        self.x_max = x2
        self.y_max = y2
        
        # Get depth information
        self.bbox_depth = float(msg.data[7])

    def get_camera_info(self):
        """
        Get camera information from the RealSense camera_info topic
        
        Returns:
            tuple: (fx, fy) focal lengths or (None, None) if not available
        """
        try:
            camera_info_topic = f'/{self.robot_name}/camera/color/camera_info'
            camera_info = rospy.wait_for_message(
                camera_info_topic,
                sensor_msgs.msg.CameraInfo,
                timeout=2.0
            )
            
            # Extract focal lengths
            fx = camera_info.K[0]
            fy = camera_info.K[4]
            
            return fx, fy
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get camera info: {e}")
            return None, None
    
    # ---------- Center Calculation and Adjustment Functions ----------
    
    def calculate_bbox_center(self):

        """
        Calculate the center of the bounding box
        
        Returns:
            tuple: (x_center, y_center) coordinates
        """
        x_center = (self.x_min + self.x_max) / 2
        y_center = (self.y_min + self.y_max) / 2

        return x_center, y_center
    
    def calculate_center_of_image(self):
        """
        Calculate the center (bottom fourth) of the image
        
        Returns:
            tuple: (x_center, y_center) coordinates
        """
        x_center = self.image_width / 2
        y_center = self.image_height * 4 / 5

        return x_center, y_center
    
    def calculate_error(self):
        """
        Calculate both horizontal and vertical errors between 
        the bounding box center and image center
        
        Returns:
            tuple: (horizontal_error, vertical_error) or (None, None) if no bbox
        """
        bbox_center = self.calculate_bbox_center()
        image_center = self.calculate_center_of_image()

        horizontal_error = bbox_center[0] - image_center[0]
        vertical_error = bbox_center[1] - image_center[1]

        return horizontal_error, vertical_error
    
    def calculate_gain(self):
        """
        Calculate both horizontal and vertical gains based on camera focal lengths
        
        Returns:
            tuple: (horizontal_gain, vertical_gain)
        """
        fx, fy = self.get_camera_info()

        if fx is None or fy is None:
            rospy.logwarn("Camera focal lengths not available, using default gains")
            return 1.0, 1.0
        
        
        horizontal_gain = 1.0 / fx if fx else 0
        vertical_gain = 1.0 / fy if fy else 0
        
        rospy.loginfo(f"Gains - horizontal: {horizontal_gain}, vertical: {vertical_gain}")
        return horizontal_gain, vertical_gain
    
    def adjust_gain_by_depth(self, gain):
        """
        Adjust gain based on depth information 
        
        Args:
            gain (float): Original gain value
            
        Returns:
            float: Depth-adjusted gain
        """
        if self.bbox_depth is not None and self.bbox_depth > 0:
            # Scale by depth - normalized to 1 meter reference distance
            return gain * (self.bbox_depth / 1.0)
        return gain
    
    def check_if_centered(self, horizontal_error, vertical_error, 
                         h_threshold=None, v_threshold=None):
        """
        Check if object is centered within thresholds
        
        Args:
            horizontal_error (float): Horizontal error in pixels
            vertical_error (float): Vertical error in pixels
            h_threshold (float): Horizontal threshold (optional)
            v_threshold (float): Vertical threshold (optional)
            
        Returns:
            bool: True if centered, False otherwise
        """
        h_threshold = h_threshold if h_threshold is not None else self.horizontal_threshold
        v_threshold = v_threshold if v_threshold is not None else self.vertical_threshold
        
        is_centered = (abs(horizontal_error) <= h_threshold and 
                      abs(vertical_error) <= v_threshold)
        
        if is_centered:
            rospy.loginfo("Object is centered in the camera view")
        else:
            if abs(vertical_error) > v_threshold:
                rospy.loginfo("Object is not centered vertically")
            if abs(horizontal_error) > h_threshold:
                rospy.loginfo("Object is not centered horizontally")
                
        return is_centered
    
    # ---------- Camera Tilt Functions ----------
    
    
    def tilt_camera(self, angle):
        """
        Tilt the camera to a specific angle
        
        Args:
            angle (float): Angle in radians to tilt the camera
        """
        try:
            camera_msg = JointGroupCommand()
            camera_msg.name = 'camera'
            camera_msg.cmd = [0.0, angle]
            self.camera_pub.publish(camera_msg)
            rospy.loginfo(f"Camera tilted to {angle} radians")
            rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr(f"Failed to tilt camera: {e}")

    def reset_camera_tilt(self):
        """Reset the camera tilt to the default position"""
        self.tilt_camera(self.default_camera_tilt)

    # ---------- Base Adjustment Functions ----------
        
    def adjust_base_orientation(self):
        """
        Adjust the robot's base orientation to center the object horizontally
        
        Returns:
            bool: True if adjustment succeeded, False otherwise
        """
        horizontal_error, _ = self.calculate_error()
        
        if horizontal_error is None:
            rospy.logwarn("Horizontal error is not available")
            return False
            
        # Check if already within tolerance
        if abs(horizontal_error) <= self.horizontal_tolerance:
            rospy.loginfo("Object is centered horizontally within tolerance")
            return True
            
        # Calculate and adjust gain
        horizontal_gain, _ = self.calculate_gain()
        adjusted_gain = self.adjust_gain_by_depth(horizontal_gain)
        
        # Calculate rotation angle with damping
        rotation_angle = -horizontal_error * adjusted_gain * self.horizontal_damping
        
        # Apply minimum threshold to avoid very small movements
        if abs(rotation_angle) < self.min_rotation_threshold:
            rospy.loginfo("Rotation too small, skipping adjustment")
            return True
            
        # Limit maximum rotation angle for safety
        rotation_angle = max(min(rotation_angle, self.max_rotation), -self.max_rotation)
        
        # Execute the rotation
        return self._execute_rotation(rotation_angle)
        
    def _execute_rotation(self, rotation_angle):
        """
        Execute a rotation of the robot base
        
        Args:
            rotation_angle (float): Angle to rotate in radians
            
        Returns:
            bool: True if rotation succeeded, False otherwise
        """
        rospy.loginfo(f"Rotating base by {rotation_angle} radians to center object")
        
        # Create quaternion for rotation
        quat = Quaternion(*quaternion_from_euler(0, 0, rotation_angle))
        
        # Get current robot position
        robot_pose = self.nav_controller.get_robot_pose()
        
        # Use navigation controller to execute rotation in place
        success = self.nav_controller.move_to_position(
            robot_pose.position.x,
            robot_pose.position.y, 
            quat,
            timeout=5.0
        )
        
        if not success:
            rospy.logerr("Failed to rotate the robot")
            return False
            
        # Wait for sensors to update
        rospy.sleep(0.5)
        return True
        
    def adjust_base_position(self):
        """
        Adjust the robot's base position to center the object vertically
        
        Returns:
            bool: True if adjustment succeeded, False otherwise
        """
        _, vertical_error = self.calculate_error()
        
        if vertical_error is None:
            rospy.logwarn("Vertical error is not available")
            return False
            
        # Check if already within tolerance
        if abs(vertical_error) <= self.center_threshold:
            rospy.loginfo("Object is centered vertically within tolerance")
            return True
            
        # Calculate and adjust gain
        _, vertical_gain = self.calculate_gain()
        adjusted_gain = self.adjust_gain_by_depth(vertical_gain)
        
        # Calculate vertical movement with damping
        vertical_distance = -vertical_error * adjusted_gain * self.vertical_damping
        
        # Limit maximum movement
        vertical_distance = max(min(vertical_distance, self.max_vertical_distance), 
                               -self.max_vertical_distance)
        
        # Apply minimum threshold
        if abs(vertical_distance) < self.min_movement_threshold:
            rospy.loginfo("Vertical movement too small, skipping adjustment")
            return True
            
        # Execute the movement
        return self._execute_movement(vertical_distance)
    
    # ---------- Movement Execution Functions ----------
        
    def _execute_movement(self, vertical_distance):
        """
        Execute a forward/backward movement of the robot base
        
        Args:
            vertical_distance (float): Distance to move in meters
            
        Returns:
            bool: True if movement succeeded, False otherwise
        """
        rospy.loginfo(f"Moving base by {vertical_distance} meters to center object")
        
        # Get current robot position
        robot_pose = self.nav_controller.get_robot_pose()
        
        # Calculate new position
        new_x = robot_pose.position.x + vertical_distance
        new_y = robot_pose.position.y
        
        # Use navigation controller to execute movement
        success = self.nav_controller.move_to_position(
            new_x,
            new_y,
            timeout=20.0
        )
        
        if not success:
            rospy.logerr("Failed to move the robot")
            return False
            
        # Wait for sensors to update
        rospy.sleep(0.5)
        return True
    
    # ---------- Fine Approach Function ----------
        
    def fine_approach(self, max_attempts=5, tilt_angle=0.75):
        """
        Perform fine approach to center object in camera view,
        prioritizing the correction of the larger error first
        
        Args:
            max_attempts (int): Maximum number of centering attempts
            tilt_angle (float): Camera tilt angle in radians for approach
            
        Returns:
            bool: True if approach succeeded, False otherwise
        """
        # Tilt camera down for better view during approach
        self.tilt_camera(tilt_angle)
        rospy.sleep(1.0)  # Wait for camera to stabilize
        
        for attempt in range(max_attempts):
            rospy.loginfo(f"Fine approach attempt {attempt+1}/{max_attempts}")
            
            # Calculate current errors
            horizontal_error, vertical_error = self.calculate_error()
            
            # Check if already centered
            if self.check_if_centered(horizontal_error, vertical_error):
                rospy.loginfo("Object is already centered")
                return True
            
            # Decide which error to fix first based on relative magnitude
            # Compare errors relative to their thresholds to decide which is "worse"
            rel_horizontal_error = abs(horizontal_error) / self.horizontal_threshold
            rel_vertical_error = abs(vertical_error) / self.vertical_threshold
            
            if rel_horizontal_error > rel_vertical_error:
                # Fix horizontal error first (rotation)
                rospy.loginfo("Horizontal error is larger, adjusting orientation first")
                self.adjust_base_orientation()
                
                # Check if centered after horizontal adjustment
                horizontal_error, vertical_error = self.calculate_error()
                if self.check_if_centered(horizontal_error, vertical_error):
                    rospy.loginfo("Fine approach completed after orientation adjustment")
                    return True
                    
                # Now fix vertical error
                self.adjust_base_position()
            else:
                # Fix vertical error first (forward/backward movement)
                rospy.loginfo("Vertical error is larger, adjusting position first")
                self.adjust_base_position()
                
                # Check if centered after vertical adjustment
                horizontal_error, vertical_error = self.calculate_error()
                if self.check_if_centered(horizontal_error, vertical_error):
                    rospy.loginfo("Fine approach completed after position adjustment")
                    return True
                    
                # Now fix horizontal error
                self.adjust_base_orientation()
            
            # Final check if centered after both adjustments
            horizontal_error, vertical_error = self.calculate_error()
            if self.check_if_centered(horizontal_error, vertical_error):
                rospy.loginfo("Fine approach completed successfully")
                return True
                
            rospy.loginfo("Object is not centered, continuing to next attempt...")
            rospy.sleep(1.0)
            
        # Reset camera tilt even if approach wasn't successful
        rospy.logwarn("Failed to center object after maximum attempts")
        return False