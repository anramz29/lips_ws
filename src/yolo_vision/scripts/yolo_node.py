#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
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

        self.model_path = rospy.get_param('~model_path')
        self.image_topic = rospy.get_param('~image_topic')
        self.depth_image_topic = rospy.get_param('~depth_image_topic')  
        self.annotation_topic = rospy.get_param('~annotated_image_topic')
        self.bbox_topic = rospy.get_param('~bbox_depth_topic')  # Parameter for bounding boxes
        
        # Load YOLO model
        self.model = YOLO(self.model_path)

        # Optimization: Set model parameters
        self.model.conf = 0.5  # Confidence threshold
        self.model.iou = 0.45  # NMS IOU threshold
        self.model.max_det = 1  # Maximum detections per image

        self.last_depth_msg = None  # Store the last depth image

        # Processing rate control
        self.last_process_time = rospy.Time.now()
        self.min_process_interval = rospy.Duration(0.1)  # 10 Hz maximum

        # Setup ROS communication
        self._setup_ros_communication()

    # ---------- ROS Communication Setup ----------

    def _setup_ros_communication(self):
        # Subscribe with queue_size=1 to drop frames if processing is slow
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, 
                                         queue_size=1, buff_size=2**24)
        
        # Subscribe to depth images
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback,
                                         queue_size=1, buff_size=2**24)
        
        # Publishers
        self.image_pub = rospy.Publisher(self.annotation_topic, Image, queue_size=1)
        self.bbox_pub = rospy.Publisher(self.bbox_topic, Float32MultiArray, queue_size=1)
        
    # ---------- Callback Functions ----------

    def depth_callback(self, msg):
        """Callback for depth images."""
        try:
            self.last_depth_msg = msg
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {str(e)}")

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
            
            # Draw detections on the frame and collect bounding boxes
            bboxes = self.annotate_detections(frame, result)
            
            # Publish the annotated image
            self.publish_annotated_image(frame, ros_image.header)
            
            # Publish bounding boxes
            self.publish_bboxes(bboxes, ros_image.header)
            
            # Update process time
            self.last_process_time = rospy.Time.now()
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    # ---------- Depth Processing ----------

    def convert_depth_image_to_cv2(self, ros_image):
        """Convert ROS depth image to meters in OpenCV format."""
        if ros_image is None:
            rospy.logwarn("Depth image is None")
            return np.zeros((480, 640), dtype=np.float32)  # Default size, adjust as needed
            
        try:
            depth = self.bridge.imgmsg_to_cv2(ros_image, '16UC1')
            depth = depth.astype(float) / 1000.0  # Convert from mm to meters
            return depth
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {str(e)}")
            return np.zeros((480, 640), dtype=np.float32)
    
    def process_bbox_depth(self, depth, bbox):
        """
        Calculate the average depth for a bounding box.
        
        Args:
            depth: OpenCV depth image in meters
            bbox: Tuple of (x, y, width, height)
            
        Returns:
            float: Average depth in meters or 0 if no valid depth
        """
        x, y, w, h = bbox
        
        # Safety checks
        if depth is None or depth.size == 0:
            return 0.0
            
        # Ensure coordinates are within image bounds
        h_max, w_max = depth.shape[:2]
        x = max(0, min(x, w_max-1))
        y = max(0, min(y, h_max-1))
        w = min(w, w_max - x)
        h = min(h, h_max - y)
        
        if w <= 0 or h <= 0:
            return 0.0
            
        # Extract depth values within bounding box
        roi_depth = depth[y:y+h, x:x+w]
        
        # Calculate average of non-zero depth values
        valid_depths = roi_depth[roi_depth > 0.1]  # Filter out very small values too
        if len(valid_depths) > 0:
            return float(np.mean(valid_depths))
        else:
            return 0.0

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

    def annotate_depth(self, img, x, y, w, h, depth):
        """Add depth text to the image at the bounding box."""
        if depth <= 0:
            depth_text = "Unknown"
        else:
            depth_text = f"{depth:.2f}m"
            
        text_size = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        
        # Calculate text position
        text_x = x + (w - text_size[0]) // 2
        text_y = y - 10 if y > 30 else y + 30
        
        # Draw text on image
        cv2.putText(img, depth_text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    def annotate_detections(self, frame, result):
        """
        Draw bounding boxes and labels for detected objects.
        
        Args:
            frame: OpenCV image to annotate
            result: YOLO detection results
            
        Returns:
            list: List of bounding boxes in format [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
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
            
            # Calculate width and height
            w, h = x2 - x1, y2 - y1
            
            # Get depth if available
            avg_depth = 0.0
            if self.last_depth_msg is not None:
                try:
                    depth = self.convert_depth_image_to_cv2(self.last_depth_msg) 
                    avg_depth = self.process_bbox_depth(depth, (x1, y1, w, h))
                    self.annotate_depth(frame, x1, y1, w, h, avg_depth)
                except Exception as e:
                    rospy.logerr(f"Error processing depth: {str(e)}")
            
            # Add detection to bboxes with depth
            bboxes.extend([cls_id, conf, x1, y1, x2, y2, avg_depth])
            
            # Get class name
            cls_name = self.get_class_name(cls_id)
            
            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add label with class name and confidence
            self.add_label_to_bbox(frame, x1, y1, cls_name, conf)
        
        return bboxes

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
        node = YoloNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass