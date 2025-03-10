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


from object_scout.utils import get_robot_pose

from object_scout.navigation_controller import NavigationController


class FineApproach():
    """
    Class for fine approach of objects
    """
    def __init__(self, robot_name, nav_controller, init_node=False):

        if init_node:
            rospy.init_node('fine_approach', anonymous=False)

        # Get ROS parameters
        self.robot_name = robot_name
        self.nav_controller = nav_controller

        # Set up depth subscription
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 
                                               f'/{robot_name}/camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )

        # Create publisher for camera joint group
        self.camera_pub = rospy.Publisher(
            f'/{self.robot_name}/commands/joint_group', 
            JointGroupCommand, 
            queue_size=1
        )


        self.approach_min_depth = 0.3
        self.approach_max_depth = 0.5
        self.navigation_timeout = 30.0
        self.default_camera_tilt = 0.2618
        self.image_height = 480
        self.image_width = 640

        self.bbox_depth = None
        self.y_min, self.x_min, self.y_max, self.x_max = 0, 0, 0, 0

    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg: Float32MultiArray containing depth data
        """
        if msg.data and len(msg.data) >= 5:
            # Extract the bounding box coordinates (x, y, w, h)
            x, y, w, h = msg.data[:4]

            # Convert to format (x_min, y_min, x_max, y_max)
            self.y_min = float(y)
            self.x_min = float(x)
            self.y_max = float(y + h)
            self.x_max = float(x + w)

            # Store the depth
            self.bbox_depth = msg.data[4]  
        else:
            rospy.logwarn("Received empty or invalid depth message")

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
        
    def calculate_bbox_center(self):
        """
        Calculate the center of the bounding box
        """
        if self.bbox_depth is None:
            rospy.logwarn("Bounding box depth is not set")
            return None

        x_center = (self.x_min + self.x_max) / 2
        y_center = (self.y_min + self.y_max) / 2

        return x_center, y_center
    
    def calculate_center_of_image(self):
        """
        Calculate the center of the image
        """
        x_center = self.image_width / 2
        y_center = self.image_height / 2

        return x_center, y_center
    
    def calculate_vertical_error(self):
        """
        Calculate the vertical error between the bounding box center and image center
        """
        bbox_center = self.calculate_bbox_center()
        image_center = self.calculate_center_of_image()

        if bbox_center is None:
            rospy.logwarn("Bounding box center is not set")
            return None

        vertical_error = bbox_center[1] - image_center[1]

        return vertical_error
    
    def calculate_horizontal_error(self):
        """
        Calculate the horizontal error between the bounding box center and image center
        """
        bbox_center = self.calculate_bbox_center()
        image_center = self.calculate_center_of_image()

        if bbox_center is None:
            rospy.logwarn("Bounding box center is not set")
            return None

        horizontal_error = bbox_center[0] - image_center[0]

        return horizontal_error
    
    def calculate_tilt_gain(self):
        """
        Calculate the tilt gain based on the vertical error
        """
        _, fy = self.get_camera_info()

        # Calculate the vertical error
        if fy is None:
            rospy.logwarn("Focal length (fy) is not available")
            return 0
        
        tilt_gain = 1.0 / fy

        rospy.loginfo(f"Tilt gain: {tilt_gain}")
        return tilt_gain
    
    def calculate_horizontal_gain(self):
        """
        Calculate the horizontal gain based on the horizontal error
        """
        fx, _ = self.get_camera_info()

        # Calculate the horizontal error
        if fx is None:
            rospy.logwarn("Focal length (fx) is not available")
            return 0
        
        horizontal_gain = 1.0 / fx

        rospy.loginfo(f"Horizontal gain: {horizontal_gain}")
        return horizontal_gain
    
    def check_preconditions(self, error_value, error_name):
        """
        Check common preconditions for adjustments
        
        Args:
            error_value: The calculated error value
            error_name: Name of the error for logging

            
        Returns:
            bool: True if preconditions are met, False otherwise
        """
        if error_value is None:
            rospy.logwarn(f"{error_name} is not set")
            return False
        
        if self.bbox_depth is None:
            rospy.logwarn("Bounding box depth is not set")
            return False
        
        return True

    def get_current_camera_tilt(self):
        """
        Get the current camera tilt angle
        
        Returns:
            float: Current tilt angle in radians or None if not available
        """
        try:
            # Subscribe to joint states topic to get current positions
            joint_states_topic = f'/{self.robot_name}/joint_states'
            
            # Use a one-time subscriber with timeout to get the current state
            joint_states_msg = rospy.wait_for_message(
                joint_states_topic,
                sensor_msgs.msg.JointState,
                timeout=2.0
            )
            
            # Find the index of the camera tilt joint in the joint_states message
            # The camera tilt joint is typically named something like 'camera_tilt'
            # or 'tilt' - you'll need to check the exact name for your robot
            tilt_joint_name = 'camera_tilt'  # Adjust this name to match your robot
            
            if tilt_joint_name in joint_states_msg.name:
                tilt_index = joint_states_msg.name.index(tilt_joint_name)
                current_tilt = joint_states_msg.position[tilt_index]
                rospy.loginfo(f"Current camera tilt: {current_tilt} radians")
                return current_tilt
            else:
                rospy.logwarn(f"Joint '{tilt_joint_name}' not found in joint states")
                return None
                
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get current camera tilt: {e}")
            return None 
        
    def tilt_camera(self, angle):
        """
        Function that tilts the camera to a specific angle

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
  
    def adjust_tilt_to_bbox(self):
        """
        Adjust the camera tilt to center the object vertically in the camera view
        
        Returns:
            bool: True if adjustment succeeded, False otherwise
        """
        # Calculate vertical error
        vertical_error = self.calculate_vertical_error()

        # Check preconditions
        if not self.check_preconditions(vertical_error, "Vertical error"):
            return False
        
        # Allow a tolerance of 10 pixels
        tolerance = 10
        
        if abs(vertical_error) <= tolerance:
            rospy.loginfo("Object is centered vertically within tolerance")
            return True
        
        # Get current tilt position
        current_tilt = self.get_current_camera_tilt()
        if current_tilt is None:
            rospy.logwarn("Current camera tilt is not available")
            return False
        
        # Calculate tilt gain
        tilt_gain = self.calculate_tilt_gain()

        # Calculate new tilt angle with proportional control
        new_tilt = current_tilt + (vertical_error * tilt_gain)
        
        # Clamp to valid range
        max_tilt = 1.5  # radians
        min_tilt = -1.5 # radians
        new_tilt = max(min_tilt, min(max_tilt, new_tilt))
        
        # Only move if the change is significant
        if abs(new_tilt - current_tilt) > 0.01:  # Minimum change threshold (radians)
            self.tilt_camera(new_tilt)
            rospy.loginfo(f"Adjusted camera tilt from {current_tilt} to {new_tilt} radians")
            return True
        else:
            rospy.loginfo("Camera tilt adjustment too small, maintaining current position")
            return True
        
    def adjust_base_orientation_to_bbox(self):
        """
        Adjust the robot's base orientation to center the object horizontally in the camera view
        
        Returns:
            bool: True if adjustment succeeded, False otherwise
        """
        # Calculate horizontal error
        horizontal_error = self.calculate_horizontal_error()
        
        # Check preconditions
        if not self.check_preconditions(horizontal_error, "Horizontal error"):
            return False
        
        # Allow a tolerance of 20 pixels (typically wider than vertical tolerance)
        tolerance = 20
        
        if abs(horizontal_error) <= tolerance:
            rospy.loginfo("Object is centered horizontally within tolerance")
            return True
        
        # Calculate horizontal gain
        horizontal_gain = self.calculate_horizontal_gain()
        
        # If depth information is available, adjust gain based on distance
        # Objects further away need larger rotation for the same pixel movement
        if self.bbox_depth is not None and self.bbox_depth > 0:
            # Scale by depth - normalized to 1 meter reference distance
            adjusted_gain = horizontal_gain * (self.bbox_depth / 1.0)
        else:
            adjusted_gain = horizontal_gain
        
        # Calculate rotation angle (negative since positive horizontal_error
        # means object is to the right, so robot should rotate counter-clockwise)
        rotation_angle = -horizontal_error * adjusted_gain
        
        # Add minimum rotation threshold to avoid very small movements
        if abs(rotation_angle) < 0.02:  # radians, about 1.15 degrees
            rospy.loginfo("Rotation too small, skipping adjustment")
            return True
        
        # Limit maximum rotation angle for safety
        max_rotation = 0.5  # radians, about 28.6 degrees
        rotation_angle = max(min(rotation_angle, max_rotation), -max_rotation)
        
        # Execute the rotation using the navigation controller
        rospy.loginfo(f"Rotating base by {rotation_angle} radians to center object")
        
        # Create quaternion for rotation (rotate in place)
        quat = Quaternion(*quaternion_from_euler(0, 0, rotation_angle))
        
        # Get current robot position
        robot_pose = get_robot_pose()
            
        # Use the navigation controller to execute the rotation in place
        success = self.nav_controller.move_to_position(
            robot_pose.position.x,
            robot_pose.position.y, 
            quat,
            timeout=5.0
        )
        
        if not success:
            rospy.logerr("Failed to rotate the robot")
            return False
        
        # Wait briefly for the rotation to complete and sensors to update
        rospy.sleep(0.5)
        
        return True
       
    def fine_approach(self):
        """
        Execute fine approach to an object by continuously adjusting camera tilt,
        base orientation, and distance until reaching the target range
        
        Returns:
            bool: True if approach succeeded, False otherwise
        """
        rospy.loginfo("Starting fine approach to object...")
        
        # Reset camera to default position first
        self.tilt_camera(self.default_camera_tilt)
        rospy.sleep(1.0)  # Allow camera to move and scene to stabilize
        
        # Set timeout for approach
        start_time = rospy.Time.now()
        timeout = rospy.Duration(self.navigation_timeout)
        
        # Main approach loop
        while not rospy.is_shutdown():
            # Check timeout
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Fine approach timed out")
                return False
            
            # Check if we have depth information
            if self.bbox_depth is None:
                rospy.logwarn("No depth information available, waiting...")
                rospy.sleep(0.5)
                continue
            
            # Log current distance
            rospy.loginfo(f"Current distance to object: {self.bbox_depth:.3f} meters")
            
            # Check if we've reached the target distance
            if self.approach_min_depth <= self.bbox_depth <= self.approach_max_depth:
                rospy.loginfo(f"Target distance reached: {self.bbox_depth:.3f} meters")
                # One final adjustment to center the object
                self.adjust_tilt_to_bbox()
                self.adjust_base_orientation_to_bbox()
                return True
            
            # Adjust camera tilt and base orientation to keep object centered
            tilt_success = self.adjust_tilt_to_bbox()
            orient_success = self.adjust_base_orientation_to_bbox()
            
            if not (tilt_success and orient_success):
                rospy.logwarn("Failed to adjust camera or orientation, retrying...")
                rospy.sleep(1.0)
                continue
            
            # Calculate approach distance
            distance_error = self.bbox_depth - ((self.approach_min_depth + self.approach_max_depth) / 2)
            
            # Determine movement direction and distance
            if abs(distance_error) < 0.05:  # Small tolerance to prevent tiny movements
                rospy.loginfo("Distance nearly optimal, fine tuning position...")
            elif distance_error > 0:  # Too far, move forward
                # Calculate forward movement (could be proportional to error)
                forward_distance = min(0.2, distance_error / 2)  # Limit to 20cm steps
                rospy.loginfo(f"Moving forward {forward_distance:.2f} meters")
                self.nav_controller.create_navigation_goal(forward_distance, 0, 0)
            else:  # Too close, move backward
                # Calculate backward movement (could be proportional to error)
                backward_distance = min(0.2, abs(distance_error) / 2)  # Limit to 20cm steps
                rospy.loginfo(f"Moving backward {backward_distance:.2f} meters")
                self.nav_controller.create_navigation_goal(-backward_distance, 0, 0)
            
            # Wait for movement to complete and sensors to update
            rospy.sleep(1.0)
        
        return False
    

        

        






    


    