#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class DistanceNode:
    """
    Node for calculating the depth of objects detected by YOLO.
    Processes annotated YOLO images and corresponding depth images,
    extracts bounding boxes, and calculates average depth within each box.
    """
    def __init__(self):
        rospy.init_node('distance_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters from parameter server
        self.yolo_image_topic = rospy.get_param('~annotated_image_topic')
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        self.visualization_topic = rospy.get_param('~visualization_topic')
        
        # Publishers
        self.viz_pub = rospy.Publisher(self.visualization_topic, Image, queue_size=10)
        self.bbox_depth_pub = rospy.Publisher(self.bbox_depth_topic, Float32MultiArray, queue_size=10)
        
        # Subscribers
        self.yolo_sub = rospy.Subscriber(self.yolo_image_topic, Image, self.yolo_callback)
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        
        # Message storage
        self.last_yolo_msg = None
        self.last_depth_msg = None

    # ---------- Callback Functions ----------

    def yolo_callback(self, msg):
        """Callback for YOLO annotated images."""
        try:
            self.last_yolo_msg = msg
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in YOLO callback: {str(e)}")

    def depth_callback(self, msg):
        """Callback for depth images."""
        try:
            self.last_depth_msg = msg
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {str(e)}")

    # ---------- Main Processing Function ----------

    def process_images(self):
        """Process the latest YOLO and depth images to extract depth for detected objects."""
        # Skip if we don't have both images
        if self.last_yolo_msg is None or self.last_depth_msg is None:
            return

        try:
            # Convert images to OpenCV format
            img = self.convert_yolo_image_to_cv2()
            depth = self.convert_depth_image_to_cv2()
            
            # Extract bounding box from annotated image
            bbox = self.get_bbox_from_mask(img)
            
            # Process depth for bounding box if found
            if bbox is not None:
                self.process_bbox_depth(img, depth, bbox)
            
            # Publish visualization
            self.publish_visualization(img)
            
        except Exception as e:
            rospy.logerr(f"Error in process_images: {str(e)}")

    # ---------- Image Conversion Functions ----------

    def convert_yolo_image_to_cv2(self):
        """Convert ROS YOLO annotated image to OpenCV format."""
        return self.bridge.imgmsg_to_cv2(self.last_yolo_msg, 'bgr8')

    def convert_depth_image_to_cv2(self):
        """Convert ROS depth image to meters in OpenCV format."""
        if self.last_depth_msg.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(self.last_depth_msg, '32FC1')
            # Convert from mm to m if needed
            if np.max(depth) > 100:
                depth = depth / 1000.0
        else:
            depth = self.bridge.imgmsg_to_cv2(self.last_depth_msg, '16UC1')
            depth = depth.astype(float) / 1000.0
        
        return depth

    # ---------- Bounding Box Functions ----------

    def get_bbox_from_mask(self, img):
        """
        Get single bounding box from the green pixels in the image.
        
        Args:
            img: OpenCV image containing green bounding box
            
        Returns:
            tuple: (x, y, width, height) of the bounding box or None if not found
        """
        # Create binary mask of green pixels
        mask = cv2.inRange(img, (0, 255, 0), (0, 255, 0))
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Return None if no contours found
        if not contours:
            return None
            
        # Find the largest contour (main bounding box)
        largest_contour = max(contours, key=cv2.contourArea)
        return cv2.boundingRect(largest_contour)

    def process_bbox_depth(self, img, depth, bbox):
        """
        Calculate and annotate the average depth for a bounding box.
        
        Args:
            img: OpenCV RGB image to annotate
            depth: OpenCV depth image in meters
            bbox: Tuple of (x, y, width, height)
        """
        x, y, w, h = bbox
        
        # Extract depth values within bounding box
        roi_depth = depth[y:y+h, x:x+w]
        
        # Calculate average of non-zero depth values
        valid_depths = roi_depth[roi_depth != 0]
        if len(valid_depths) > 0:
            avg_depth = np.mean(valid_depths)
            
            # Publish depth information with bbox coordinates
            self.publish_depth_data(x, y, w, h, avg_depth)
            
            # Add depth text to the image
            self.annotate_depth(img, x, y, w, h, avg_depth)

    # ---------- Publishing Functions ----------

    def publish_depth_data(self, x, y, w, h, depth):
        """Publish bounding box coordinates and depth."""
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(w), float(h), depth]
        self.bbox_depth_pub.publish(msg)

    def annotate_depth(self, img, x, y, w, h, depth):
        """Add depth text to the image at the bounding box."""
        depth_text = f"{depth:.2f}m"
        text_size = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        
        # Calculate text position
        text_x = x + (w - text_size[0]) // 2
        text_y = y - 10 if y > 30 else y + 30
        
        # Draw text on image
        cv2.putText(img, depth_text, (text_x, text_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    def publish_visualization(self, img):
        """Publish the annotated image for visualization."""
        viz_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.viz_pub.publish(viz_msg)


if __name__ == '__main__':
    try:
        node = DistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass