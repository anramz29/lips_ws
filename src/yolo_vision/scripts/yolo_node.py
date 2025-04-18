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
        self.device = rospy.get_param('~device', 'cuda:0')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.2)

        # Check CUDA availability
        import torch
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            rospy.logwarn("CUDA requested but not available. Falling back to CPU.")
            self.device = 'cpu'
        else:
            rospy.loginfo(f"Using device: {self.device}")
            if self.device.startswith('cuda'):
                rospy.loginfo(f"CUDA device: {torch.cuda.get_device_name(0)}")
        
        # Load YOLO model
        self.model = YOLO(self.model_path)
        self.model.to(self.device)
        

        # Processing rate control
        self.last_process_time = rospy.Time.now()

        # use 40 hz as default
        self.min_process_interval = rospy.Duration(0.1)

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
                           imgsz=640,
                           device=self.device)    # Reduce image size
        
        return next(results)  # Get first result from generator

    def process_detections(self, result):
        """
        Process detection results and only return the bounding box with highest confidence.
        
        Args:
            result: YOLO detection results
            
        Returns:
            list: List containing single highest confidence bounding box in format 
                [n_boxes, cls_id, conf, x1, y1, x2, y2]
                If no detections meet the threshold, returns [0]
        """
        # Initialize with 0 detections
        bboxes = [0]  # First element is number of boxes
        best_det = None
        best_conf = 0.0
        
        if len(result.boxes) > 0:
            # Iterate over detected boxes
            for det in result.boxes:
                # if confidence is below threshold, skip detection
                if det.conf[0] < self.confidence_threshold:
                    continue
                
                # Get confidence
                conf = float(det.conf[0])
                
                # Update best detection if confidence is higher
                if conf > best_conf:
                    best_conf = conf
                    best_det = det
        
        # If we found a valid detection
        if best_det is not None:
            # Get box coordinates of best detection
            box = best_det.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = box
            cls_id = int(best_det.cls[0])
            
            # Update number of boxes to 1
            bboxes[0] = 1
            
            # Add best detection details
            bboxes.extend([cls_id, best_conf, x1, y1, x2, y2])
        
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
    