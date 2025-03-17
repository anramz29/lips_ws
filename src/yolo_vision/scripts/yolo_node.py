#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloNode:
    """
    Node for running YOLO object detection on camera images.
    Processes incoming images, runs inference with YOLO, 
    and publishes annotated images with bounding boxes.
    """
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters
        model_path = rospy.get_param("~model_path")
        image_topic = rospy.get_param("~image_topic")
        annotation_topic = rospy.get_param("~annotated_image_topic")

        # Load YOLO model
        self.model = YOLO(model_path)

        # Optimization: Set model parameters
        self.model.conf = 0.5  # Confidence threshold
        self.model.iou = 0.45  # NMS IOU threshold
        self.model.max_det = 1  # Maximum detections per image
        
        # Subscribe with queue_size=1 to drop frames if processing is slow
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, 
                                         queue_size=1, buff_size=2**24)
        
        # Publisher
        self.image_pub = rospy.Publisher(annotation_topic, Image, queue_size=1)
        
        # Processing rate control
        self.last_process_time = rospy.Time.now()
        self.min_process_interval = rospy.Duration(0.1)  # 10 Hz maximum

    # ---------- Callback Functions ----------

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
            
            # Draw detections on the frame
            self.annotate_detections(frame, result)
            
            # Publish the annotated image
            self.publish_annotated_image(frame, ros_image.header)
            
            # Update process time
            self.last_process_time = rospy.Time.now()
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    # ---------- Processing Functions ----------

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

    # ---------- Visualization Functions ----------

    def annotate_detections(self, frame, result):
        """
        Draw bounding boxes and labels for detected objects.
        
        Args:
            frame: OpenCV image to annotate
            result: YOLO detection results
        """
        if len(result.boxes) > 0:
            # Get the most confident detection
            det = result.boxes[0]  # Only process the first detection
            
            # Get box coordinates and confidence
            box = det.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = box
            conf = float(det.conf[0])
            cls_id = int(det.cls[0])
            
            # Get class name
            cls_name = self.get_class_name(cls_id)
            
            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add label with class name and confidence
            self.add_label_to_bbox(frame, x1, y1, cls_name, conf)

    def get_class_name(self, cls_id):
        """
        Get the name of a class from its ID.
        
        Args:
            cls_id: Class ID
            
        Returns:
            str: Class name
        """
        if hasattr(self.model, 'names') and cls_id < len(self.model.names):
            return self.model.names[cls_id]
        else:
            return str(cls_id)

    def add_label_to_bbox(self, frame, x, y, cls_name, conf):
        """
        Add a label to a bounding box.
        
        Args:
            frame: OpenCV image
            x, y: Top-left corner of bounding box
            cls_name: Class name
            conf: Confidence score
        """
        label = f"{cls_name} {conf:.2f}"
        label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        
        # Ensure label is within image bounds
        text_y = max(y, label_size[1] + 5)
        
        cv2.putText(frame, label, (x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ---------- Publishing Functions ----------

    def publish_annotated_image(self, frame, header):
        """
        Publish an annotated image.
        
        Args:
            frame: Annotated OpenCV image
            header: Original image header
        """
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = header
        self.image_pub.publish(annotated_msg)

    def spin(self):
        """
        Run the node.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YoloNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass