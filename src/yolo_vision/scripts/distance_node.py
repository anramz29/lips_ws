#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from collections import deque

class SynchronizedDistanceNode:
    """
    Node for calculating distances from depth images using bounding boxes from YOLO.
    Uses synchronized subscribers to ensure RGB, depth, and bounding box data correspond to the same frame.
    """
    def __init__(self):
        rospy.init_node('synchronized_distance_node', anonymous=True)
        self.bridge = CvBridge()

        # Get parameters
        self.image_topic = rospy.get_param('~image_topic')
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.bbox_topic = rospy.get_param('~bbox_topic')  
        self.annotation_topic = rospy.get_param('~annotated_image_topic')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        
        # Sync parameters
        self.sync_queue_size = rospy.get_param('~sync_queue_size')
        self.sync_slop = rospy.get_param('~sync_slop')  
        
        # Setup ROS communication
        self._setup_ros_communication()

    def _setup_ros_communication(self):
        # Publishers
        self.image_pub = rospy.Publisher(self.annotation_topic, Image, queue_size=1)
        self.bbox_depth_pub = rospy.Publisher(self.bbox_depth_topic, Float32MultiArray, queue_size=1)
        
        # Set up synchronized subscribers
        self._setup_synchronized_subscribers()
    
    def _setup_synchronized_subscribers(self):
        """Set up message_filters to synchronize incoming messages."""
        # Create subscribers with message_filters for synchronization
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_image_topic, Image)
        
        # Special handling for Float32MultiArray which doesn't have a header
        bbox_sub = message_filters.Subscriber(self.bbox_topic, Float32MultiArray)
        
        # TimeSynchronizer requires headers on all messages
        # ApproximateTimeSynchronizer allows for non-exact timestamp matches
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, bbox_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        
        # Register the callback for synchronized messages
        self.ts.registerCallback(self.synchronized_callback)
        
        rospy.loginfo("Synchronized subscribers set up successfully")

    def synchronized_callback(self, rgb_msg, depth_msg, bbox_msg):
        """
        Process synchronized RGB image, depth image, and bounding box data.
        All messages should have approximately the same timestamp.
        
        Args:
            rgb_msg: RGB image message
            depth_msg: Depth image message
            bbox_msg: Bounding box message
        """
        rospy.logdebug(f"Received synchronized data with timestamps: "
                     f"RGB: {rgb_msg.header.stamp}, Depth: {depth_msg.header.stamp}")
        
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.convert_depth_image_to_cv2(depth_msg)
            
            # Process bounding boxes and calculate distances
            bbox_depth_data = self.process_bboxes_with_depth(rgb_frame, depth_image, bbox_msg.data)
            
            # Publish the annotated image
            self.publish_annotated_image(rgb_frame, rgb_msg.header)
            
            # Publish bounding boxes with depth
            self.publish_bbox_depth(bbox_depth_data, rgb_msg.header)
            
        except Exception as e:
            rospy.logerr(f"Error in processing synchronized data: {str(e)}")

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
        x1 = max(0, min(int(x1), w_max-1))
        y1 = max(0, min(int(y1), h_max-1))
        x2 = min(int(x2), w_max-1)
        y2 = min(int(y2), h_max-1)
        
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
        Process bounding boxes with depth information.
        If a single detection is invalid, it skips only that detection instead of all.
        
        Args:
            frame: RGB frame to annotate
            depth_image: Depth image in meters
            bbox_data: Array of bounding box data
            
        Returns:
            list: Bounding box data with depth information
        """
        # Convert to list for easy manipulation
        bbox_data = list(bbox_data)
        
        # Check for valid data
        if not bbox_data:
            rospy.logwarn("Received empty bounding box data")
            return [0]  # No boxes
            
        num_boxes = int(bbox_data[0])
        
        if num_boxes == 0:
            return [0]  # No boxes to process
            
        # Expected data length
        expected_length = 1 + num_boxes * 6
        if len(bbox_data) < expected_length:
            rospy.logwarn(f"Expected at least {expected_length} elements for {num_boxes} boxes, got {len(bbox_data)}")
            # Adjust num_boxes to what we can process with available data
            num_boxes = (len(bbox_data) - 1) // 6
            if num_boxes == 0:
                return [0]  # Not enough data for even one box
        
        # Create new bbox data with depth
        bbox_depth_data = [num_boxes]
        valid_boxes = 0
        
        # Process each box
        for i in range(num_boxes):
            try:
                start_idx = 1 + i * 6
                if start_idx + 5 >= len(bbox_data):
                    break  # Not enough data for this box
                    
                cls_id = int(bbox_data[start_idx])
                conf = bbox_data[start_idx + 1]
                x1 = int(bbox_data[start_idx + 2])
                y1 = int(bbox_data[start_idx + 3])
                x2 = int(bbox_data[start_idx + 4])
                y2 = int(bbox_data[start_idx + 5])
                
                # Skip boxes with invalid coordinates
                if x1 >= x2 or y1 >= y2 or x1 < 0 or y1 < 0:
                    rospy.logwarn(f"Skipping box with invalid coordinates: [{x1}, {y1}, {x2}, {y2}]")
                    continue
                
                avg_depth = self.process_bbox_depth(depth_image, (x1, y1, x2, y2))
                bbox_depth_data.extend([cls_id, conf, x1, y1, x2, y2, avg_depth])
                self.annotate_frame(frame, cls_id, conf, x1, y1, x2, y2, avg_depth)
                valid_boxes += 1
                
            except Exception as e:
                rospy.logwarn(f"Error processing box {i}: {str(e)}")
                continue
        
        # Update the count to reflect valid boxes
        bbox_depth_data[0] = valid_boxes
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

    def publish_annotated_image(self, frame, header):
        """
        Publish the annotated RGB image.
        
        Args:
            frame: Annotated OpenCV frame
            header: Header to use for the message
        """
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing annotated image: {str(e)}")

    def publish_bbox_depth(self, bbox_depth_data, header):
        """
        Publish bounding boxes with depth information.
        
        Args:
            bbox_depth_data: List of bounding box data with depth
            header: Header to use for the message
        """
        try:
            # Create message
            msg = Float32MultiArray()
            
            # Add a custom header field to the layout
            msg.layout.data_offset = header.stamp.to_nsec()
            
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
        node = SynchronizedDistanceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass