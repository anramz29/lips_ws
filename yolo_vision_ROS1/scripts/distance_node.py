#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class DistanceNode:
    def __init__(self):
        rospy.init_node('distance_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters from parameter server
        self.yolo_image_topic = rospy.get_param('~yolo_image_topic')
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        
        # Publishers
        self.viz_pub = rospy.Publisher('camera/yolo/visualization', Image, queue_size=10)
        self.bbox_depth_pub = rospy.Publisher(self.bbox_depth_topic, Float32MultiArray, queue_size=10)
        
        # Subscribers
        self.yolo_sub = rospy.Subscriber(self.yolo_image_topic, Image, self.yolo_callback)
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        
        self.last_yolo_msg = None
        self.last_depth_msg = None

    def yolo_callback(self, msg):
        try:
            self.last_yolo_msg = msg
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in YOLO callback: {str(e)}")

    def depth_callback(self, msg):
        try:
            self.last_depth_msg = msg
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {str(e)}")

    def get_bbox_from_mask(self, img):
        """Get single bounding box from the green pixels in the image."""
        mask = cv2.inRange(img, (0, 255, 0), (0, 255, 0))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        # Find the largest contour (main bounding box)
        largest_contour = max(contours, key=cv2.contourArea)
        return cv2.boundingRect(largest_contour)

    def process_images(self):
        if self.last_yolo_msg is None or self.last_depth_msg is None:
            return

        try:
            # Get the annotated image (with YOLO's bounding box)
            img = self.bridge.imgmsg_to_cv2(self.last_yolo_msg, 'bgr8')
            
            # Convert depth image
            if self.last_depth_msg.encoding == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(self.last_depth_msg, '32FC1')
                if np.max(depth) > 100:
                    depth = depth / 1000.0
            else:
                depth = self.bridge.imgmsg_to_cv2(self.last_depth_msg, '16UC1')
                depth = depth.astype(float) / 1000.0
            
            # Get the single bounding box from the YOLO annotation
            bbox = self.get_bbox_from_mask(img)
            
            if bbox is not None:
                x, y, w, h = bbox
                roi_depth = depth[y:y+h, x:x+w]
                
                valid_depths = roi_depth[roi_depth != 0]
                if len(valid_depths) > 0:
                    avg_depth = np.mean(valid_depths)
                    
                    # Create message with bbox coordinates and depth
                    msg = Float32MultiArray()
                    msg.data = [float(x), float(y), float(w), float(h), avg_depth]
                    self.bbox_depth_pub.publish(msg)
                    
                    # Add depth text to the existing image
                    depth_text = f"{avg_depth:.2f}m"
                    text_size = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
                    text_x = x + (w - text_size[0]) // 2
                    text_y = y - 10 if y > 30 else y + 30
                    
                    cv2.putText(img, depth_text, (text_x, text_y), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in process_images: {str(e)}")

if __name__ == '__main__':
    try:
        node = DistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass