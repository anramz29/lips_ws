#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool, Float32
from std_srvs.srv import SetBool, SetBoolResponse


class YoloKeypointDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_keypoint_detection_node', anonymous=True)
        
        # Global parameters
        self.debug_mode = rospy.get_param('~debug_mode')
        self.is_enabled = rospy.get_param('~is_enabled', False)

        # YoLo model parameters
        self.model_path = rospy.get_param('~model')
        self.device = rospy.get_param('~device')
        self.input_size = rospy.get_param('~input_size')
        self.conf_threshold = rospy.get_param('~confidence_threshold')

        # Robot name parameter (for service/topic namespace)
        self.robot_name = rospy.get_param('~robot_name', 'locobot')

        # input topics
        self.image_topic = rospy.get_param('~image_topic')

        # Output topics
        self.keypoints_topic = rospy.get_param('~keypoints_topic', 'camera/yolo/keypoints')
        self.keypoints_viz_topic = rospy.get_param('~keypoints_viz_topic', 'camera/yolo/keypoints_visualization')
        self.object_angle_topic = rospy.get_param('~object_angle_topic', 'camera/yolo/object_angle')

        # Intialize Image processing bridge
        self.bridge = CvBridge()
        
        # State variables
        self.latest_depth = None
        self.last_process_time = rospy.Time(0)
        self.process_interval = rospy.Duration(1.0 / 10)  # 10 Hz max

        self.model = YOLO(self.model_path)

        # Setup ROS communication
        self._setup_ros_communication()

    def _setup_ros_communication(self):
        # Subscribe to RGB images
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback, 
            queue_size=1
        )

        # Publishers
        self.keypoints_pub = rospy.Publisher(
            self.keypoints_topic,
            Float32MultiArray,
            queue_size=1
        )

        self.keypoints_viz_pub = rospy.Publisher(
            self.keypoints_viz_topic,
            Image,
            queue_size=1
        )

        self.object_angle_pub = rospy.Publisher(
            self.object_angle_topic,
            Float32,
            queue_size=1
        )

        # Subscribe to enable/disable topic
        self.enable_sub = rospy.Subscriber(
            f'/{self.robot_name}/keypoint_detector/enable',
            Bool,
            self.enable_callback,
            queue_size=1
        )

        # Create service for direct access
        self.service = rospy.Service(
            f'/{self.robot_name}/keypoint_detector/set_enabled',
            SetBool,
            self.handle_set_enabled
        )

        rospy.loginfo(f"YOLO keypoint detection node initialized. Detection is currently {'enabled' if self.is_enabled else 'disabled'}.")

    def enable_callback(self, msg):
        """Callback for the enable/disable topic"""
        self.is_enabled = msg.data
        state = "enabled" if self.is_enabled else "disabled"
        rospy.loginfo(f"Keypoint detection {state}")

    def handle_set_enabled(self, req):
        """Handle service request to enable/disable keypoint detection"""
        self.is_enabled = req.data
        
        state = "enabled" if self.is_enabled else "disabled"
        rospy.loginfo(f"Keypoint detection {state}")
        
        # Return success response
        return SetBoolResponse(
            success=True,
            message=f"Keypoint detection {state}"
        )

    def run_model(self, image):
        """
        Run the YOLO model on the input image.
        
        Args:
            image: np.array, input image
        
        Returns:
            results: dict, model output
        """
        results = self.model(image,
                            imgsz=self.input_size, 
                            device=self.device,
                            stream=True,
                            verbose=False)
        
        return next(results)
    
    def calculate_angle(self, keypoints):
        """
        Calculate the angle between consecutive keypoints with respect to the y-axis,
        normalized to the (-90, 90) range, as well as the perpendicular angle.
        
        Args:
            keypoints: np.array, shape (N, 3) with x, y, confidence for each keypoint
            
        Returns:
            angles: list of values containing both the angle_deg and perpendicular_angle_deg
        """
        angles = []

        # Need at least 2 keypoints to calculate an angle
        if len(keypoints) < 2:
            return angles
            
        # Calculate angles between consecutive keypoints
        for i in range(len(keypoints) - 1):
            if keypoints[i, 2] > 0.1 and keypoints[i+1, 2] > 0.1:  # Only use valid keypoints
                # Calculate angle with respect to x-axis first
                dx = keypoints[i+1, 0] - keypoints[i, 0]
                dy = keypoints[i+1, 1] - keypoints[i, 1]
                
                # For y-axis reference, we need to calculate the angle from the vertical
                # We can use arctan2(dx, dy) instead of arctan2(dy, dx)
                # This effectively rotates our reference by 90 degrees
                angle_rad = np.arctan2(dx, dy)
                
                # Normalize to (-pi/2, pi/2) range
                if angle_rad > np.pi/2:
                    angle_rad = angle_rad - np.pi
                elif angle_rad < -np.pi/2:
                    angle_rad = angle_rad + np.pi
                
                angle_deg = np.degrees(angle_rad)
                
                # Calculate perpendicular angle (rotate by 90 degrees)
                perp_angle_deg = angle_deg + 90.0
                # Normalize to (-90, 90) range
                if perp_angle_deg > 90:
                    perp_angle_deg -= 180
                
                # Only store the angle in degrees and the perpendicular angle
                angles.append(angle_deg)
                angles.append(perp_angle_deg)

        return angles
    
    def process_data(self, results):
        """
        Process the results of the YOLO model.
        
        Args:
            results: dict, model output
        
        Returns:
            tuple: (processed_results, perpendicular_angle) or (None, None) if no valid results
        """
        # Check if we have any results
        if len(results) == 0:
            return None, None  # Return tuple of None values
        
        # Extract data from results
        keypoints = results.keypoints.data if results.keypoints is not None else None
        if keypoints is None or len(keypoints) == 0:
            return None, None  # Return tuple of None values
            
        class_ids = results.boxes.cls
        confidences = results.boxes.conf
        
        processed_results = []
        perpendicular_angle = None
        
        # Process each detection
        for i in range(len(class_ids)):
            if confidences[i].item() < self.conf_threshold:
                continue

            # Create an entry for this detection: [class_id, confidence, keypoints...]
            detection = [class_ids[i].item(), confidences[i].item()]
            
            # Filter keypoints based on confidence
            valid_keypoints = []
            keypoints_data = keypoints[i]
            
            # Process each keypoint, filtering out low confidence or zero-coordinate points
            valid_keypoints_array = []
            for kp_idx, kp in enumerate(keypoints_data):
                x, y, conf = kp
                x_val, y_val, conf_val = x.item(), y.item(), conf.item()
                
                # Skip points with very low confidence or zero coordinates
                if conf_val < 0.1 or (abs(x_val) < 0.01 and abs(y_val) < 0.01):
                    continue
                    
                valid_keypoints.extend([x_val, y_val, conf_val])
                valid_keypoints_array.append([x_val, y_val, conf_val])
            
            # Convert to numpy array for angle calculation
            valid_keypoints_array = np.array(valid_keypoints_array)
            
            # Calculate angles between consecutive keypoints (simplified to just degrees)
            angles = self.calculate_angle(valid_keypoints_array)
            
            # Safely get perpendicular angle if available
            if len(angles) >= 2:
                perpendicular_angle = angles[0]  # Every other angle is the perpendicular angle
            
            # Add number of valid keypoints and the keypoints themselves
            detection.append(len(valid_keypoints) // 3)  # Number of valid keypoints
            detection.extend(valid_keypoints)
            
            # Add angles data to the detection list
            detection.append(len(angles))  # Number of angles
            detection.extend(angles)      # The actual angle values
            
            processed_results.append(detection)
        
        return processed_results, perpendicular_angle
    
    def publish_keypoints(self, processed_results):
        """
        Publishes keypoints as Float32MultiArray and visualization image
        
        Args:
            processed_results: List of [class_id, conf, num_keypoints, keypoints...] for each detection
        """
        if not processed_results:
            return
        
        # Create Float32MultiArray message
        msg = Float32MultiArray()
        
        # First value is number of detections
        all_data = [len(processed_results)]
        
        # Flatten the list of detections
        for detection in processed_results:
            all_data.extend(detection)
        
        msg.data = all_data
        self.keypoints_pub.publish(msg)

    def publish_object_angle(self, angle):
        """
        Publishes the object angle as a Float32 message
        
        Args:
            angle: float, angle value
        """
        msg = Float32()
        msg.data = angle
        self.object_angle_pub.publish(msg)
        
    def create_viz_image(self, image, processed_results):
        """
        Create a visualization image with keypoints and angles
        
        Args:
            image: np.array, input image
            processed_results: List of [class_id, conf, num_keypoints, keypoints..., num_angles, angles...] for each detection
        
        Returns:
            np.array, visualization image
        """
        if not processed_results:
            return image
        
        for detection in processed_results:
            # Extract basic info
            class_id = detection[0]
            confidence = detection[1]
            num_keypoints = detection[2]
            
            if num_keypoints < 2:
                continue  # Skip if not enough keypoints for angle calculation
                
            # Extract keypoints - starting after class_id, confidence, and num_keypoints
            keypoints_flat = detection[3:3 + num_keypoints * 3]
            keypoints = np.array(keypoints_flat).reshape(-1, 3)
            
            # Calculate average confidence
            avg_conf = np.mean(keypoints[:, 2])
            
            # Draw only the valid keypoints
            for kp_idx, kp in enumerate(keypoints):
                x, y, conf = kp
                if conf > 0.1:  # Additional filter for safety
                    cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
                    cv2.putText(image, f"{kp_idx}", (int(x)+5, int(y)+5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Get the number of angles from the processed results
            # The index after keypoints data
            angles_index = 3 + num_keypoints * 3
            num_angles = detection[angles_index]
            
            # Extract angle values (now just degrees)
            angles = []
            if num_angles > 0:
                angles_start_idx = angles_index + 1  # Skip the num_angles value
                angles = detection[angles_start_idx:angles_start_idx + num_angles]
            
            # Draw connecting lines and display angles
            if num_angles > 0:
                for i in range(len(keypoints) - 1):
                    if keypoints[i, 2] > 0.1 and keypoints[i+1, 2] > 0.1:  # Only use valid keypoints
                        # Draw line between the two keypoints
                        cv2.line(image, 
                                (int(keypoints[i, 0]), int(keypoints[i, 1])),
                                (int(keypoints[i+1, 0]), int(keypoints[i+1, 1])),
                                (0, 255, 0), 2)
                        
                        # Show angle between points
                        if i < len(angles):
                            mid_x = int((keypoints[i, 0] + keypoints[i+1, 0]) / 2)
                            mid_y = int((keypoints[i, 1] + keypoints[i+1, 1]) / 2) - 40
                            cv2.putText(image, f"{angles[i]:.1f}°", (mid_x, mid_y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
            # Add overall stats
            cv2.putText(image, f"Avg KP Conf: {avg_conf:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image, f"Valid KPs: {num_keypoints}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image, f"Angles: {num_angles}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return image
    
    def image_callback(self, msg):
        """
        Callback for RGB image messages
        
        Args:
            msg: Image, RGB image message
        """
        # Skip processing if the node is disabled
        if not self.is_enabled:
            return
            
        # Convert image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run the model
        results = self.run_model(image)

        # Process the results
        processed_results, perpendicular_angle = self.process_data(results)

        # Only publish if we have valid results
        if processed_results is not None:
            # Publish keypoints
            self.publish_keypoints(processed_results)

            # Publish object angle if valid
            if perpendicular_angle is not None:
                self.publish_object_angle(perpendicular_angle)

            # Create visualization image
            viz_image = self.create_viz_image(image.copy(), processed_results)
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
            self.keypoints_viz_pub.publish(viz_msg)
        else:
            # Publish empty visualization image when no results
            viz_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.keypoints_viz_pub.publish(viz_msg)


if __name__ == '__main__':
    node = YoloKeypointDetectionNode()
    rospy.spin()