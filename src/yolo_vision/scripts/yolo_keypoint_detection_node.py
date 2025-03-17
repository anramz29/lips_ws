#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO


class KeypointDetectionNode:
    """
    ROS node for detecting keypoints in images using YOLOv8.
    Detects and publishes keypoints from camera images.
    """
    def __init__(self):
        rospy.init_node('keypoint_detection_node', anonymous=True)
        
        # Load parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.image_topic = rospy.get_param('~image_topic', '/locobot/camera/color/image_raw')
        self.keypoints_topic = rospy.get_param('~keypoints_topic', 'keypoints')
        self.keypoints_viz_topic = rospy.get_param('~keypoints_viz_topic', 'keypoints_visualization')
        self.model_path = rospy.get_param('~model')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.input_size = rospy.get_param('~input_size', 640)
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self._init_detector()
        
        # Set up ROS communication
        self._setup_ros_communication()
        
        rospy.loginfo("Keypoint detection node initialized successfully")
    
    def _init_detector(self):
        """Initialize the YOLOv8 keypoint detection model."""
        try:
            self.detector = YOLO(self.model_path)
            rospy.loginfo(f"Loaded YOLOv8 keypoint model from: {self.model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize YOLO keypoint model: {e}")
            raise
    
    def _setup_ros_communication(self):
        """Set up ROS publishers and subscribers."""
        # Subscribers
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24  # Larger buffer for images
        )
        
        # Publishers
        self.keypoints_pub = rospy.Publisher(
            self.keypoints_topic, 
            Float32MultiArray, 
            queue_size=10
        )
        self.keypoints_viz_pub = rospy.Publisher(
            self.keypoints_viz_topic, 
            Image, 
            queue_size=10
        )

    def image_callback(self, msg):
        """
        Process incoming images and detect keypoints.
        
        Args:
            msg (sensor_msgs.msg.Image): Input image message
        """
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect keypoints
            keypoints, confidence = self.detect_keypoints(cv_image)
            
            # Create visualization
            viz_image = self.visualize_keypoints(cv_image, keypoints, confidence)
            
            # Publish results
            self.publish_keypoints(keypoints)
            self.keypoints_viz_pub.publish(self.bridge.cv2_to_imgmsg(viz_image, "bgr8"))
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


    def detect_keypoints(self, image):
        """
        Detect keypoints using YOLOv8 model with streaming.
        
        Args:
            image (numpy.ndarray): Input RGB image
                
        Returns:
            tuple: (keypoints, confidences) where:
                - keypoints is a numpy.ndarray with shape (N, 2)
                - confidences is a numpy.ndarray with shape (N,)
        """
        try:
            # Process image with YOLOv8 in streaming mode
            results = self.detector(image, stream=True, conf=self.conf_threshold)
            
            all_keypoints = []
            all_confidences = []
            
            # Iterate through all results
            for result in results:
                if hasattr(result, 'keypoints') and result.keypoints is not None:
                    keypoints_data = result.keypoints.data
                    
                    # Process each detection in this result
                    for detection in keypoints_data:
                        # Get the keypoint coordinates
                        kps = detection[:, :2].cpu().numpy()
                        
                        # Filter keypoints by confidence if available (3rd column is confidence)
                        if detection.shape[1] > 2:
                            kp_confidences = detection[:, 2].cpu().numpy()
                            
                            # Add all keypoints with confidence > 0.5
                            for i, (kp, conf) in enumerate(zip(kps, kp_confidences)):
                                if conf > 0.5:  # 50% confidence threshold
                                    all_keypoints.append(kp)
                                    all_confidences.append(conf)
                                    if self.debug_mode:
                                        rospy.loginfo(f"Detection keypoint {i}: confidence {conf:.3f}")
                        else:
                            # If no confidence values, add all keypoints with default confidence
                            all_keypoints.extend(kps)
                            all_confidences.extend([1.0] * len(kps))  # Default confidence of 1.0
                            
            if self.debug_mode:
                rospy.loginfo(f"Selected {len(all_keypoints)} keypoints from all detections")
            return np.array(all_keypoints, dtype=np.float32), np.array(all_confidences, dtype=np.float32)
                
        except Exception as e:
            rospy.logerr(f"Error detecting keypoints: {e}")
            return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

        
    def visualize_keypoints(self, image, keypoints, confidences=None):
        """
        Create a visualization of the detected keypoints with confidence values.
        
        Args:
            image (numpy.ndarray): Original RGB image
            keypoints (numpy.ndarray): Array of keypoint coordinates
            confidences (numpy.ndarray): Optional array of confidence values
            
        Returns:
            numpy.ndarray: Image with visualized keypoints
        """
        # Create a copy of the image for visualization
        viz_image = image.copy()
        
        # Check if we detected any keypoints
        if len(keypoints) == 0:
            cv2.putText(viz_image, "No keypoints detected", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return viz_image
        
        # Draw each keypoint
        for i, kp in enumerate(keypoints):
            x, y = int(kp[0]), int(kp[1])
            
            # Skip if coordinates are outside the image
            if x < 0 or y < 0 or x >= image.shape[1] or y >= image.shape[0]:
                continue
                
            # Get confidence if available
            conf_text = ""
            color = (0, 255, 0)  # Default green
            
            if confidences is not None and i < len(confidences):
                conf = confidences[i]
                conf_text = f" {conf:.2f}"
                
                # Color based on confidence (green to red)
                # High confidence: green, Low confidence: red
                g = int(255 * conf)
                r = int(255 * (1 - conf))
                color = (0, g, r)
            
            # Draw keypoint as a circle with size based on confidence
            radius = 5
            if confidences is not None and i < len(confidences):
                radius = max(3, int(7 * confidences[i]))
            
            cv2.circle(viz_image, (x, y), radius, color, -1)
            
            # Label each keypoint with index and confidence
            cv2.putText(viz_image, f"{i}{conf_text}", (x + 5, y - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Add information text
        cv2.putText(viz_image, f"Detected {len(keypoints)} keypoints", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return viz_image

    def publish_keypoints(self, keypoints, confidences=None):
        """
        Publish detected keypoints as a Float32MultiArray.
        
        Args:
            keypoints (numpy.ndarray): Array of keypoint coordinates
            confidences (numpy.ndarray): Optional array of confidence values
        """
        msg = Float32MultiArray()
        
        # Format: [num_keypoints, x1, y1, conf1, x2, y2, conf2, ...]
        msg.data = [float(len(keypoints))]
        
        if len(keypoints) > 0:
            # Include both coordinates and confidences in the message
            if confidences is not None and len(confidences) == len(keypoints):
                for i, kp in enumerate(keypoints):
                    msg.data.extend([float(kp[0]), float(kp[1]), float(confidences[i])])
            else:
                # Fallback if no confidences available
                flattened = keypoints.reshape(-1)
                msg.data.extend([float(coord) for coord in flattened])
        
        self.keypoints_pub.publish(msg)

if __name__ == '__main__':
    try:
        node = KeypointDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")