#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO

class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters
        model_path = rospy.get_param("~model_path")
        self.model = YOLO(model_path)
        
        # Optimization: Set model parameters
        self.model.conf = 0.5  # Confidence threshold
        self.model.iou = 0.45  # NMS IOU threshold
        self.model.max_det = 1  # Maximum detections per image
        
        # If using CUDA, optimize for GPU
        if torch.cuda.is_available():
            self.model.to('cuda')
            torch.backends.cudnn.benchmark = True
        
        # Subscribe with queue_size=1 to drop frames if processing is slow
        image_topic = rospy.get_param("~image_topic")
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, 
                                        queue_size=1, buff_size=2**24)
        
        # Publisher
        self.image_pub = rospy.Publisher('/locobot/camera/yolo/annotated_image', 
                                       Image, queue_size=1)
        
        # Processing rate control
        self.last_process_time = rospy.Time.now()
        self.min_process_interval = rospy.Duration(0.1)  # 10 Hz maximum
        
        rospy.loginfo("YoloNode initialized.")

    def image_callback(self, ros_image):
        # Rate limiting
        current_time = rospy.Time.now()
        if (current_time - self.last_process_time) < self.min_process_interval:
            return
        
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            
            # Run YOLO inference with optimized parameters
            results = self.model(frame, 
                               verbose=False,
                               stream=True,  # Enable streaming mode
                               imgsz=640)    # Reduce image size
            
            # Process results
            result = next(results)  # Get first result from generator
            
            if len(result.boxes) > 0:
                # Get the most confident detection
                det = result.boxes[0]  # Only process the first detection
                
                # Get box coordinates and confidence
                box = det.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = box
                conf = float(det.conf[0])
                cls_id = int(det.cls[0])
                
                # Get class name if available
                if hasattr(self.model, 'names') and cls_id < len(self.model.names):
                    cls_name = self.model.names[cls_id]
                else:
                    cls_name = str(cls_id)
                
                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                
                # Add label with class name and confidence
                label = f"{cls_name} {conf:.2f}"
                label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                
                # Ensure label is within image bounds
                text_y = max(y1, label_size[1] + 5)
                cv2.putText(frame, label, (x1, text_y), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
            # Convert and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = ros_image.header
            self.image_pub.publish(annotated_msg)
            
            # Update process time
            self.last_process_time = current_time
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YoloNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass