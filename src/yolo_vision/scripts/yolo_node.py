#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetectionNode:
    """
    Node for running YOLO object detection on camera images.
    Processes incoming images, runs inference with YOLO, and publishes only bounding boxes.
    No depth calculation or image annotation is performed by this node.
    """
    def __init__(self):
        rospy.init_node('yolo_detection_node', anonymous=True)
        self.bridge = CvBridge()

        # Get parameters
        self.model_path = rospy.get_param('~model_path')
        self.image_topic = rospy.get_param('~image_topic')
        self.bbox_topic = rospy.get_param('~bbox_topic')
        
        # Load YOLO model
        self.model = YOLO(self.model_path)

        # Optimization: Set model parameters
        self.model.conf = 0.5  # Confidence threshold
        self.model.iou = 0.45  # NMS IOU threshold
        self.model.max_det = 1  # Maximum detections per image

        # Processing rate control
        self.last_process_time = rospy.Time.now()
        self.min_process_interval = rospy.Duration(0.1)  # 10 Hz maximum

        # Setup ROS communication
        self._setup_ros_communication()

    def _setup_ros_communication(self):
        # Subscribe with queue_size=1 to drop frames if processing is slow
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24
        )
        
        # Publisher for bounding boxes
        self.bbox_pub = rospy.Publisher(
            self.bbox_topic, 
            Float32MultiArray, 
            queue_size=1
        )
        
    def image_callback(self, ros_image):
        """
        Process incoming images with YOLO object detection.
        
        Args:
            ros_image: ROS Image message
        """
        # Rate limiting
        if not self.should_process_image():
            return
        
        try:
            # Convert ROS Image to OpenCV image
            frame = self.convert_to_cv_image(ros_image)
            
            # Run YOLO inference
            result = self.run_yolo_inference(frame)
            
            # Process detections and collect bounding boxes
            bboxes = self.process_detections(result)
            
            # Publish bounding boxes
            self.publish_bboxes(bboxes, ros_image.header)
            
            # Update process time
            self.last_process_time = rospy.Time.now()
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    def should_process_image(self):
        """
        Check if we should process this image based on rate limiting.
        
        Returns:
            bool: True if the image should be processed, False otherwise
        """
        current_time = rospy.Time.now()
        return (current_time - self.last_process_time) >= self.min_process_interval

    def convert_to_cv_image(self, ros_image):
        """
        Convert ROS Image message to OpenCV image.
        
        Args:
            ros_image: ROS Image message
            
        Returns:
            numpy.ndarray: OpenCV image
        """
        return self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

    def run_yolo_inference(self, frame):
        """
        Run YOLO inference on an image.
        
        Args:
            frame: OpenCV image
            
        Returns:
            object: YOLO results
        """
        results = self.model(frame, 
                           verbose=False,
                           stream=True,  # Enable streaming mode
                           imgsz=640)    # Reduce image size
        
        return next(results)  # Get first result from generator

    def process_detections(self, result):
        """
        Process detection results and prepare bounding box data for publishing.
        
        Args:
            result: YOLO detection results
            
        Returns:
            list: List of bounding boxes in format [n_boxes, cls_id, conf, x1, y1, x2, y2]
        """
        # Initialize bboxes array with number of detections
        bboxes = [len(result.boxes)]  # First element is number of boxes
        
        if len(result.boxes) > 0:
            # Get the most confident detection
            det = result.boxes[0]  # Only process the first detection
            
            # Get box coordinates and confidence
            box = det.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = box
            conf = float(det.conf[0])
            cls_id = int(det.cls[0])
            
            # Add detection to bboxes
            bboxes.extend([cls_id, conf, x1, y1, x2, y2])
        
        return bboxes

    def publish_bboxes(self, bboxes, header):
        """
        Publish bounding boxes as a Float32MultiArray.
        
        Args:
            bboxes: List of bounding box data
            header: Original image header for timestamp
        """
        # Create message
        msg = Float32MultiArray()
        
        # Set dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "bboxes"
        msg.layout.dim[0].size = len(bboxes)
        msg.layout.dim[0].stride = len(bboxes)
        
        # Set data
        msg.data = bboxes
        
        # Publish message
        self.bbox_pub.publish(msg)

    def spin(self):
        """
        Run the node.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YoloDetectionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass