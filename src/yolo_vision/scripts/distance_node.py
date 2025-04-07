#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np

class DistanceNode:
    """
    Node for calculating distances from depth images using bounding boxes from YOLO.
    Subscribes to YOLO bounding boxes and depth images.
    Publishes annotated RGB images and bounding boxes with depth information.
    """
    def __init__(self):
        rospy.init_node('distance_node', anonymous=True)
        self.bridge = CvBridge()

        # Get parameters
        self.image_topic = rospy.get_param('~image_topic')
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.bbox_topic = rospy.get_param('~bbox_topic')  
        self.annotation_topic = rospy.get_param('~annotated_image_topic')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        
        # Store the latest messages
        self.latest_image = None
        self.latest_depth = None
        self.latest_bbox = None
        self.latest_image_header = None
        
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
        
        # Subscribe to depth images
        self.depth_sub = rospy.Subscriber(
            self.depth_image_topic, 
            Image, 
            self.depth_callback,
            queue_size=1
        )
        
        # Subscribe to bounding boxes from YOLO
        self.bbox_sub = rospy.Subscriber(
            self.bbox_topic,
            Float32MultiArray,
            self.bbox_callback,
            queue_size=1
        )
        
        # Publishers
        self.image_pub = rospy.Publisher(self.annotation_topic, Image, queue_size=1)
        self.bbox_depth_pub = rospy.Publisher(self.bbox_depth_topic, Float32MultiArray, queue_size=1)
        
        # Timer for processing at a fixed rate
        self.process_timer = rospy.Timer(rospy.Duration(0.1), self.process_data)  # 10 Hz

    def image_callback(self, msg):
        """Store the latest RGB image."""
        self.latest_image = msg
        self.latest_image_header = msg.header

    def depth_callback(self, msg):
        """Store the latest depth image."""
        self.latest_depth = msg

    def bbox_callback(self, msg):
        """Store the latest bounding box data."""
        self.latest_bbox = msg

    def process_data(self, event):
        """
        Process all available data to calculate distances and annotate images.
        Called periodically by the timer.
        """
        # Check if we have all required data
        if None in (self.latest_image, self.latest_depth, self.latest_bbox):
            return
        
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            depth_image = self.convert_depth_image_to_cv2(self.latest_depth)
            
            # Process bounding boxes and calculate distances
            bbox_depth_data = self.process_bboxes_with_depth(rgb_frame, depth_image, self.latest_bbox.data)
            
            # Publish the annotated image
            self.publish_annotated_image(rgb_frame)
            
            # Publish bounding boxes with depth
            self.publish_bbox_depth(bbox_depth_data)
            
        except Exception as e:
            rospy.logerr(f"Error in processing data: {str(e)}")

    def convert_depth_image_to_cv2(self, ros_image):
        """Convert ROS depth image to meters in OpenCV format."""
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
            bbox: Tuple of (x1, y1, x2, y2)
            
        Returns:
            float: Average depth in meters or 0 if no valid depth
        """
        x1, y1, x2, y2 = bbox
        
        # Safety checks
        if depth is None or depth.size == 0:
            return 0.0
            
        # Ensure coordinates are within image bounds
        h_max, w_max = depth.shape[:2]
        x1 = max(0, min(x1, w_max-1))
        y1 = max(0, min(y1, h_max-1))
        x2 = min(x2, w_max-1)
        y2 = min(y2, h_max-1)
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
            
        # Extract depth values within bounding box
        roi_depth = depth[y1:y2, x1:x2]
        
        # Calculate average of non-zero depth values
        valid_depths = roi_depth[roi_depth > 0.1]  # Filter out very small values too
        if len(valid_depths) > 0:
            return float(np.mean(valid_depths))
        else:
            return 0.0
    def process_bboxes_with_depth(self, frame, depth_image, bbox_data):
        """
        
        """
        # Convert to list for easy manipulation
        bbox_data = list(bbox_data)
        
        # Validate input data
        if not bbox_data:
            return [0]
            
        num_boxes = int(bbox_data[0])
        
        if num_boxes == 0 or len(bbox_data) < 1 + num_boxes * 6:
            return [0]  # Not enough data for the specified number of boxes
        
        # Create new bbox data with depth
        bbox_depth_data = [num_boxes]
        
        # Process each valid box
        for i in range(num_boxes):
            start_idx = 1 + i * 6
            cls_id = int(bbox_data[start_idx])
            conf = bbox_data[start_idx + 1]
            x1 = int(bbox_data[start_idx + 2])
            y1 = int(bbox_data[start_idx + 3])
            x2 = int(bbox_data[start_idx + 4])
            y2 = int(bbox_data[start_idx + 5])
            
            avg_depth = self.process_bbox_depth(depth_image, (x1, y1, x2, y2))
            bbox_depth_data.extend([cls_id, conf, x1, y1, x2, y2, avg_depth])
            self.annotate_frame(frame, cls_id, conf, x1, y1, x2, y2, avg_depth)
        
        return bbox_depth_data
    
    
    def annotate_frame(self, frame, cls_id, conf, x1, y1, x2, y2, depth):
        """
        Annotate a frame with bounding box, class, confidence, and depth.
        
        Args:
            frame: RGB frame to annotate
            cls_id, conf, x1, y1, x2, y2, depth: Detection data
        """
        # Draw rectangle
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Add class name and confidence
        cls_name = self.get_class_name(cls_id)
        label = f"{cls_name} {conf:.2f}"
        
        # Get text sizes
        label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        
        # Calculate positions for text
        # First line (class + confidence) - place it above bounding box
        text_y1 = max(y1 - 10, label_size[1])
        
        # Second line (depth) - place it just above the first line
        text_y2 = max(y1 - 10 - label_size[1] - 5, baseline)
        
        # Draw class and confidence (first line)
        cv2.putText(frame, label, (x1, text_y1), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw depth information (second line)
        if depth <= 0:
            depth_text = "Distance: Unknown"
        else:
            depth_text = f"Distance: {depth:.2f}m"
        
        cv2.putText(frame, depth_text, (x1, text_y2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def get_class_name(self, cls_id):
        """
        Get class name for a class ID.
        This is a placeholder - in a real implementation, you would 
        have a proper mapping from class ID to name.
        
        Args:
            cls_id: Class ID
            
        Returns:
            str: Class name
        """
        # Placeholder - you should use a proper mapping
        class_names = {
            0: "box",
            1: "grasshopper"
        }
        return class_names.get(cls_id, f"class_{cls_id}")

    def publish_annotated_image(self, frame):
        """Publish the annotated RGB image."""
        if self.latest_image_header is None:
            return
            
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = self.latest_image_header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing annotated image: {str(e)}")

    def publish_bbox_depth(self, bbox_depth_data):
        """Publish bounding boxes with depth information."""
        if self.latest_image_header is None:
            return
            
        try:
            # Create message
            msg = Float32MultiArray()

            dim = MultiArrayDimension()
            dim.label = "bbox_depth"
            dim.size = len(bbox_depth_data)
            dim.stride = len(bbox_depth_data)
            msg.layout.dim.append(dim)
            
            # Set data
            msg.data = bbox_depth_data
            
            # Publish
            self.bbox_depth_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error publishing bbox depth data: {str(e)}")

    def spin(self):
        """Run the node."""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DistanceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass