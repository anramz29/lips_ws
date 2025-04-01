#!/usr/bin/env python3
import rospy
import math
import actionlib
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from interbotix_xs_msgs.msg import JointGroupCommand # type: ignore
import sensor_msgs.msg
from geometry_msgs.msg import Twist


class FineApproacher:
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

        # Centering parameters
        self.horizontal_adjustment_tolerance = 40 
        self.vertical_adjustment_tolerance = 70 
        
        # Overall centering completion criteria    
        self.horizontal_centering_threshold = 100
        self.vertical_centering_threshold = 70 


        self.max_vertical_distance = 0.5  # meters
        self.max_rotation = 0.5  # radians, about 28.6 degrees
        self.min_rotation_threshold = 0.02  # radians
        self.min_movement_threshold = 0.015  # meters
        self.horizontal_damping = 1.0
        self.vertical_damping = 1.3

   
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

        self.twist_pub = rospy.Subscriber(
            f'/mobile_base/cmd_vel', 
            Twist,
            queue_size=1
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
        h_threshold = h_threshold if h_threshold is not None else self.horizontal_centering_threshold
        v_threshold = v_threshold if v_threshold is not None else self.vertical_centering_threshold
        
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
    
    def fine_approach(self, max_duration=60, rate=10):
        """
        Continuously adjust to keep robot centered on the object
        
        Args:
            max_duration (float): Maximum time in seconds for the centering operation
            rate (int): Rate in Hz for checking position and adjusting movement
            
        Returns:
            bool: True if successfully maintained centering, False if failed or timed out
        """
        rate_obj = rospy.Rate(rate)  # Control loop rate (Hz)
        start_time = rospy.Time.now()
        
        # Variables to track if we've seen the object and been centered
        object_visible = False
        was_centered = False
        consecutive_centered_counts = 0
        required_centered_counts = 5  # Need this many consecutive centered frames to consider it stable
        
        # Last known error values for dampening
        last_h_error = 0
        last_v_error = 0
        
        while not rospy.is_shutdown():
            # Check for timeout
            if (rospy.Time.now() - start_time).to_sec() > max_duration:
                rospy.logwarn("Centering operation timed out")
                self.stop_robot()
                return False
                
            # Check if we have bbox data
            if not hasattr(self, 'x_min') or not hasattr(self, 'bbox_depth'):
                rospy.logwarn_throttle(2.0, "No bounding box detected")
                object_visible = False
                consecutive_centered_counts = 0
                self.stop_robot()
                rate_obj.sleep()
                continue
            
            object_visible = True
            
            # Calculate current errors
            horizontal_error, vertical_error = self.calculate_error()
            
            # Check if centered
            is_centered = self.check_if_centered(horizontal_error, vertical_error)
            
            if is_centered:
                consecutive_centered_counts += 1
                if consecutive_centered_counts >= required_centered_counts:
                    # We've been centered for enough consecutive frames
                    if not was_centered:
                        rospy.loginfo("Object successfully centered and stable")
                        was_centered = True
                    
                    # Maintain a very small velocity to keep tracking
                    # This helps respond quickly if the object moves
                    self._apply_minimal_tracking_velocity()
                    rate_obj.sleep()
                    continue
            else:
                # Reset counter if we lose centering
                consecutive_centered_counts = 0
                was_centered = False
            
            # Calculate movement with error smoothing
            self._move_with_error_smoothing(horizontal_error, vertical_error, last_h_error, last_v_error)
            
            # Update last error values
            last_h_error = horizontal_error
            last_v_error = vertical_error
            
            rate_obj.sleep()
        
        # Stop robot if we exit the loop
        self.stop_robot()
        return was_centered

    def _move_with_error_smoothing(self, h_error, v_error, last_h_error, last_v_error):
        """
        Move robot with smoothed error calculations to prevent overshooting
        
        Args:
            h_error (float): Current horizontal error
            v_error (float): Current vertical error
            last_h_error (float): Previous horizontal error
            last_v_error (float): Previous vertical error
        """
        # Calculate gains
        h_gain, v_gain = self.calculate_gain()
        
        # Adjust gains by depth
        h_gain = self.adjust_gain_by_depth(h_gain)
        v_gain = self.adjust_gain_by_depth(v_gain)
        
        # Apply exponential smoothing to errors to reduce oscillation
        smoothing_factor = 0.7  # Higher value = more weight on current measurement
        smoothed_h_error = smoothing_factor * h_error + (1 - smoothing_factor) * last_h_error
        smoothed_v_error = smoothing_factor * v_error + (1 - smoothing_factor) * last_v_error
        
        # Create Twist message
        cmd = Twist()
        
        # Calculate proportional component for rotation
        p_term_rot = -1 * smoothed_h_error * h_gain * self.horizontal_damping
        
        # Calculate derivative component (change in error)
        d_term_rot = -0.2 * (h_error - last_h_error) * h_gain
        
        # Combine for PD control
        cmd.angular.z = p_term_rot + d_term_rot
        
        # Limit max rotation
        cmd.angular.z = max(-self.max_rotation, min(self.max_rotation, cmd.angular.z))
        
        # Dynamic threshold based on distance - more precise when closer
        dynamic_rotation_threshold = self.min_rotation_threshold
        if self.bbox_depth and self.bbox_depth < 0.5:  # When close
            dynamic_rotation_threshold /= 2  # More precision required
        
        # If rotation is very small, don't rotate
        if abs(cmd.angular.z) < dynamic_rotation_threshold:
            cmd.angular.z = 0.0
        
        # Similar PD control for linear motion
        p_term_lin = -1 * smoothed_v_error * v_gain * self.vertical_damping
        d_term_lin = -0.2 * (v_error - last_v_error) * v_gain
        
        forward_speed = p_term_lin + d_term_lin
        
        # Limit max forward speed
        forward_speed = max(-self.max_vertical_distance, min(self.max_vertical_distance, forward_speed))
        
        # Distance-based velocity scaling - move slower when closer
        if self.bbox_depth and self.bbox_depth < 0.5:
            # Scale down speed when close to target
            forward_speed *= (self.bbox_depth / 0.5)
        
        # Dynamic threshold based on distance
        dynamic_movement_threshold = self.min_movement_threshold
        if self.bbox_depth and self.bbox_depth < 0.5:
            dynamic_movement_threshold /= 2  # More precision when closer
        
        # If movement is very small, don't move
        if abs(forward_speed) < dynamic_movement_threshold:
            forward_speed = 0.0
        
        cmd.linear.x = forward_speed
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log at lower frequency to avoid console spam
        rospy.loginfo_throttle(1.0, 
            f"Moving: linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}, " 
            f"h_error={h_error:.1f}, v_error={v_error:.1f}, depth={getattr(self, 'bbox_depth', 'unknown')}")

    def _apply_minimal_tracking_velocity(self):
        """Apply minimal velocity to maintain active tracking when centered"""
        cmd = Twist()
        
        # Very small values just to keep the system responsive
        # These should be below the movement thresholds for actual motion
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop all robot movement"""
        cmd = Twist()  # All fields default to 0
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Robot stopped")







        
        
        
