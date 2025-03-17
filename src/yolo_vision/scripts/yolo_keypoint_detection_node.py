#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool

class YoloKeypointDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_keypoint_detection_node', anonymous=True)
        
        # Get parameters
        self.model_path = rospy.get_param('~model', 'yolov8n-pose.pt')
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.input_size = rospy.get_param('~input_size', 640)
        self.debug_mode = rospy.get_param('~debug_mode', False)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        
        # Output topics
        self.keypoints_topic = rospy.get_param('~keypoints_topic', 'camera/yolo/keypoints')
        self.keypoints_viz_topic = rospy.get_param('~keypoints_viz_topic', 'camera/yolo/keypoints_visualization')
        
        # Initialize bridge and model
        self.bridge = CvBridge()
        self.model = None
        self.load_model_when_needed = True  # Lazy loading
        
        # State variables
        self.enabled = False
        self.last_process_time = rospy.Time(0)
        self.process_interval = rospy.Duration(1.0 / 10)  # 10 Hz max
        
        # Setup publishers
        self.keypoints_pub = rospy.Publisher(self.keypoints_topic, Float32MultiArray, queue_size=1)
        self.viz_pub = rospy.Publisher(self.keypoints_viz_topic, Image, queue_size=1)
        
        # Setup subscribers
        self.enable_sub = rospy.Subscriber(
            'keypoint_detector/enable',
            Bool,
            self.enable_callback
        )
        
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo(f"Keypoint detection node initialized (initially disabled)")
        rospy.loginfo(f"Will use model: {self.model_path}")
        rospy.loginfo(f"Subscribing to: {self.image_topic}")
        rospy.loginfo(f"Publishing to: {self.keypoints_topic} and {self.keypoints_viz_topic}")
    
    def enable_callback(self, msg):
        """Handle enable/disable messages"""
        if msg.data != self.enabled:
            self.enabled = msg.data
            state = "enabled" if self.enabled else "disabled"
            rospy.loginfo(f"Keypoint detection {state}")
            
            # Load model when first enabled
            if self.enabled and self.load_model_when_needed and self.model is None:
                rospy.loginfo("Loading YOLO keypoint model...")
                try:
                    self.model = YOLO(self.model_path)
                    rospy.loginfo("Model loaded successfully")
                    self.load_model_when_needed = False
                except Exception as e:
                    rospy.logerr(f"Failed to load model: {e}")
                    self.enabled = False
    
    def should_process_image(self):
        """Determine if we should process this image frame"""
        if not self.enabled:
            return False
            
        current_time = rospy.Time.now()
        if (current_time - self.last_process_time) < self.process_interval:
            return False
            
        self.last_process_time = current_time
        return True
    
    def image_callback(self, msg):
        """Process incoming images"""
        # Skip processing if disabled or too frequent
        if not self.should_process_image():
            return
            
        # Ensure model is loaded
        if self.model is None:
            if self.load_model_when_needed:
                rospy.loginfo("Lazy-loading YOLO keypoint model...")
                try:
                    self.model = YOLO(self.model_path)
                    rospy.loginfo("Model loaded successfully")
                    self.load_model_when_needed = False
                except Exception as e:
                    rospy.logerr(f"Failed to load model: {e}")
                    self.enabled = False
                    return
            else:
                # Model should be loaded, but isn't
                rospy.logerr("Model not available, but should be loaded")
                self.enabled = False
                return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process with YOLO
            results = self.model.predict(
                cv_image,
                conf=self.confidence_threshold,
                verbose=False
            )
            
            if not results or len(results) == 0:
                # No detections
                if self.debug_mode:
                    rospy.logdebug("No keypoints detected")
                return
            
            # Process results
            self.process_results(results[0], cv_image)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def process_results(self, result, image):
        """Process detection results and publish data"""
        # Create visualization image
        viz_image = image.copy()
        
        # Check if keypoints are available
        if result.keypoints is not None and len(result.keypoints.data) > 0:
            # Get keypoints data
            keypoints_data = result.keypoints.data.cpu().numpy()
            
            # Create message for detected keypoints
            msg = Float32MultiArray()
            
            # Initialize data with number of instances
            data = [float(len(keypoints_data))]
            
            # Process each detection's keypoints
            for i, keypoints in enumerate(keypoints_data):
                # For each keypoint in this detection that has confidence > threshold
                for kp in keypoints:
                    x, y, conf = kp
                    if conf > self.confidence_threshold:
                        # Add keypoint coordinates to data
                        data.append(float(x))
                        data.append(float(y))
                        
                        # Draw keypoint on visualization image
                        cv2.circle(viz_image, (int(x), int(y)), 5, (0, 255, 0), -1)
            
            # Set data in message
            msg.data = data
            
            # Publish keypoints data
            self.keypoints_pub.publish(msg)
            
            # Publish visualization
            if self.viz_pub.get_num_connections() > 0:
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "bgr8")
                viz_msg.header.stamp = rospy.Time.now()
                viz_msg.header.frame_id = self.camera_frame
                self.viz_pub.publish(viz_msg)

if __name__ == '__main__':
    try:
        node = YoloKeypointDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass