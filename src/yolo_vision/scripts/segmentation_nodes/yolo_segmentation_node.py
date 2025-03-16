#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
# Import ROS message types for publishing structured data
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D, PoseWithCovariance
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension, MultiArrayLayout, UInt8MultiArray, String
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import VisionInfo


class Yolo_Segmentation_Node():
    def __init__(self):
        rospy.init_node('yolo_segmentation_node', anonymous=True)
        self.bridge = CvBridge()

        # Load parameters
        self.model_path = rospy.get_param("~model_path")
        self.image_topic = rospy.get_param("~image_topic")
        
        # Flag to enable/disable visualization publishers
        self.publish_visualizations = rospy.get_param("~publish_visualizations", True)
        
        # Topics for visualization (optional)
        self.annotation_topic = rospy.get_param("~annotated_image_topic", "camera/yolo/annotated_image")
        self.masks_topic = rospy.get_param("~masks_topic", "camera/yolo/segmentation_masks")
        self.boxes_topic = rospy.get_param("~boxes_topic", "camera/yolo/bounding_boxes")
        
        # Topics for structured data (always needed)
        self.bbox_data_topic = rospy.get_param("~bbox_data_topic", "camera/yolo/detection_boxes")
        self.mask_data_topic = rospy.get_param("~mask_data_topic", "camera/yolo/detection_masks")
        self.class_info_topic = rospy.get_param("~class_info_topic", "camera/yolo/class_information")
        
        # Rate limiting
        self.last_process_time = rospy.Time(0)
        self.min_process_interval = rospy.Duration(1.0 / 10)  # 10 Hz max processing rate

        # Load YOLO model
        self.model = YOLO(self.model_path)

        # Optimization: Set model parameters
        self.model.conf = 0.5  # Confidence threshold   
        self.model.iou = 0.45   # NMS IOU threshold
        self.model.max_det = 10  # Maximum detections per image

        # Set up ROS communication
        self._setup_ros_communication()

    # ---------- ROS Communication Setup ----------

    def _setup_ros_communication(self):
        """Set up ROS publishers and subscribers"""
        # RGB image subscriber
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )
        
        # Publishers for visualization (optional)
        if self.publish_visualizations:
            # Annotated image publisher
            self.image_pub = rospy.Publisher(
                self.annotation_topic, Image, queue_size=1
            )
            
            # Mask image publisher
            self.masks_pub = rospy.Publisher(
                self.masks_topic, Image, queue_size=1
            )
            
            # Bounding box image publisher
            self.boxes_pub = rospy.Publisher(
                self.boxes_topic, Image, queue_size=1
            )
            
            rospy.loginfo("Visualization publishers enabled")
        else:
            rospy.loginfo("Visualization publishers disabled")
        
        # Publishers for structured data (always created)
        # Bounding box data publisher - publishes array of bounding boxes
        self.bbox_data_pub = rospy.Publisher(
            self.bbox_data_topic, Detection2DArray, queue_size=1
        )
        
        # Mask data publisher - publishes binary masks
        self.mask_data_pub = rospy.Publisher(
            self.mask_data_topic, UInt8MultiArray, queue_size=1
        )
        
        # Class information publisher - provides mapping between class IDs and names
        self.class_info_pub = rospy.Publisher(
            self.class_info_topic, VisionInfo, queue_size=1, latch=True
        )
        
        # Publish class information once at startup
        self.publish_class_information()
        
        rospy.loginfo("Yolo segmentation node initialized")

    def publish_class_information(self):
        """
        Publish information about the classes that the model can detect.
        This allows other nodes to know what class IDs correspond to what objects.
        """
        class_info = VisionInfo()
        class_info.header.stamp = rospy.Time.now()
        class_info.header.frame_id = "camera_optical_frame"  # Adjust as needed
        
        # The class information is contained in the model's names dictionary
        class_info.method = f"YOLO {self.model_path}"
        
        # Convert class dictionary to a string representation
        class_dict = self.model.names
        class_info.database_location = str(class_dict)
        
        # Publish class information
        self.class_info_pub.publish(class_info)

    # ---------- GPU Acceleration ----------

    def enable_gpu_acceleration(self):
        """
        Enable GPU acceleration for YOLO model if available.
        """
        if self.model.device.type == 'cuda':
            self.model.to('cuda')
            rospy.loginfo("GPU acceleration enabled")
        else:
            rospy.logwarn("GPU not available, using CPU")

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

    def annotate_detections(self, frame, results):
        """
        Annotate the detections on the image with masks, bounding boxes, and labels.
        
        Args:
            frame: OpenCV image (numpy array)
            results: Segmentation results from YOLO
        
        Returns:
            numpy.ndarray: Annotated image
        """
        # Skip if visualizations disabled
        if not self.publish_visualizations:
            return None
            
        # Create a copy of the frame to draw on
        annotated_frame = frame.copy()
        
        # Check if there are masks in the results
        if results.masks is not None:
            # Get segmentation masks
            masks = results.masks.data.cpu().numpy()
            
            # Get bounding boxes and class information
            boxes = results.boxes.data.cpu().numpy()
            
            # Process each detected object
            for i, (mask, box) in enumerate(zip(masks, boxes)):
                # Extract box coordinates and class
                x1, y1, x2, y2, conf, class_id = box
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                class_id = int(class_id)
                
                # Get class name
                class_name = results.names[class_id]
                
                # Create random color for this instance (consistent per class)
                color = self.get_color_for_class(class_id)
                
                # Convert mask to binary mask with proper dimensions
                binary_mask = mask.astype(np.uint8)
                
                # Apply colored mask overlay
                colored_mask = np.zeros_like(frame, dtype=np.uint8)
                colored_mask[binary_mask == 1] = color
                
                # Blend the mask with the original image (semi-transparent)
                alpha = 0.5  # Transparency factor
                mask_area = (binary_mask == 1)
                annotated_frame[mask_area] = cv2.addWeighted(
                    frame[mask_area], 1 - alpha, colored_mask[mask_area], alpha, 0
                )
                
                # Draw bounding box
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                
                # Draw label with confidence
                label = f"{class_name} {conf:.2f}"
                text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(annotated_frame, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), color, -1)
                cv2.putText(annotated_frame, label, (x1, y1 - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # Optional: Draw mask contours for better visibility
                contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(annotated_frame, contours, -1, color, 2)
        
        return annotated_frame

    def create_masks_image(self, frame, results):
        """
        Create an image showing only the segmentation masks.
        
        Args:
            frame: Original frame (for size reference)
            results: YOLO results
            
        Returns:
            numpy.ndarray: Image with only masks
        """
        # Skip if visualizations disabled
        if not self.publish_visualizations:
            return None
            
        # Create a blank image (black background)
        mask_image = np.zeros_like(frame)
        
        # Check if there are masks in the results
        if results.masks is not None:
            # Get segmentation masks
            masks = results.masks.data.cpu().numpy()
            boxes = results.boxes.data.cpu().numpy()
            
            # Process each detected object
            for i, (mask, box) in enumerate(zip(masks, boxes)):
                # Extract class
                class_id = int(box[5])
                
                # Get color for this class
                color = self.get_color_for_class(class_id)
                
                # Convert mask to binary
                binary_mask = mask.astype(np.uint8)
                
                # Apply color to mask areas
                mask_area = (binary_mask == 1)
                mask_image[mask_area] = color
        
        return mask_image

    def create_boxes_image(self, frame, results):
        """
        Create an image showing only the bounding boxes.
        
        Args:
            frame: Original frame
            results: YOLO results
            
        Returns:
            numpy.ndarray: Image with only bounding boxes
        """
        # Skip if visualizations disabled
        if not self.publish_visualizations:
            return None
            
        # Create a copy of the frame (transparent background)
        boxes_image = frame.copy()
        boxes_image = boxes_image * 0  # Black background
        
        # Get bounding boxes
        if results.boxes is not None:
            boxes = results.boxes.data.cpu().numpy()
            
            # Process each detected object
            for i, box in enumerate(boxes):
                # Extract box coordinates and class
                x1, y1, x2, y2, conf, class_id = box
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                class_id = int(class_id)
                
                # Get color for this class
                color = self.get_color_for_class(class_id)
                
                # Draw bounding box
                cv2.rectangle(boxes_image, (x1, y1), (x2, y2), color, 2)
                
                # Draw label with confidence
                label = f"{results.names[class_id]} {conf:.2f}"
                text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(boxes_image, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), color, -1)
                cv2.putText(boxes_image, label, (x1, y1 - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return boxes_image

    def create_bbox_array(self, results, header):
        """
        Create a Detection2DArray message containing bounding box information.
        
        Args:
            results: YOLO results
            header: ROS header from original image
            
        Returns:
            Detection2DArray: ROS message with bounding box data
        """
        detection_array = Detection2DArray()
        detection_array.header = header
        
        # Check if there are boxes in the results
        if results.boxes is not None:
            boxes = results.boxes.data.cpu().numpy()
            
            for box in boxes:
                # Extract box coordinates, confidence, and class
                x1, y1, x2, y2, conf, class_id = box
                x1, y1, x2, y2 = map(float, [x1, y1, x2, y2])
                class_id = int(class_id)
                
                # Create Detection2D message for this bounding box
                detection = Detection2D()
                detection.header = header
                
                # Set bounding box
                detection.bbox.center.x = (x1 + x2) / 2.0
                detection.bbox.center.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1
                
                # Set class hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = class_id
                hypothesis.score = float(conf)
                
                # Create an identity pose
                hypothesis.pose = PoseWithCovariance()
                
                # Add class name as a string in the source_img field
                # This is a bit of a hack, but allows us to include the class name
                class_name = results.names[class_id]
                
                # Add hypothesis to results
                detection.results.append(hypothesis)
                
                # Add detection to array
                detection_array.detections.append(detection)
        
        return detection_array

    def create_mask_array(self, frame, results):
        """
        Create a UInt8MultiArray message containing mask information.
        
        Args:
            frame: Original frame (for size reference)
            results: YOLO results
            
        Returns:
            UInt8MultiArray: ROS message with mask data and metadata
        """
        # Create a multi-array message
        mask_array = UInt8MultiArray()
        
        # Check if there are masks in the results
        if results.masks is not None and results.boxes is not None:
            # Get image dimensions
            height, width = frame.shape[:2]
            
            # Get masks and boxes
            masks = results.masks.data.cpu().numpy()
            boxes = results.boxes.data.cpu().numpy()
            
            # Set up the layout for the multi-array
            # First dimension: number of objects
            # Second dimension: height of the image
            # Third dimension: width of the image
            mask_array.layout.dim.append(MultiArrayDimension(
                label="objects",
                size=len(masks),
                stride=height * width
            ))
            mask_array.layout.dim.append(MultiArrayDimension(
                label="height",
                size=height,
                stride=width
            ))
            mask_array.layout.dim.append(MultiArrayDimension(
                label="width",
                size=width,
                stride=1
            ))
            
            # Set up metadata as part of the layout
            # Store class IDs and confidence scores in the data_offset field
            # Format: class_id1,conf1,class_id2,conf2,...
            metadata = []
            for box in boxes:
                class_id = int(box[5])
                conf = float(box[4])
                metadata.extend([class_id, conf])
            
            # Store metadata
            mask_array.layout.data_offset = len(metadata)
            
            # Combine all binary masks into a flattened array
            all_masks = np.zeros((len(masks), height, width), dtype=np.uint8)
            for i, mask in enumerate(masks):
                # Resize mask to match frame dimensions if needed
                if mask.shape[0] != height or mask.shape[1] != width:
                    binary_mask = cv2.resize(mask.astype(np.uint8), (width, height))
                else:
                    binary_mask = mask.astype(np.uint8)
                all_masks[i] = binary_mask * 255  # Scale to 0-255 range
            
            # Flatten the masks and add metadata
            mask_array.data = np.concatenate([
                np.array(metadata, dtype=np.uint8),
                all_masks.flatten()
            ])
        else:
            # If no masks, create an empty array with proper layout
            mask_array.layout.dim.append(MultiArrayDimension(
                label="objects",
                size=0,
                stride=0
            ))
            mask_array.layout.data_offset = 0
            mask_array.data = np.array([], dtype=np.uint8)
        
        return mask_array

    def get_color_for_class(self, class_id):
        """
        Generate a consistent color for a class ID.
        
        Args:
            class_id: Class ID
            
        Returns:
            tuple: BGR color
        """
        # Generate colors based on class ID for consistency
        colors = [
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (0, 255, 255),  # Yellow
            (255, 0, 255),  # Magenta
            (128, 0, 0),    # Dark blue
            (0, 128, 0),    # Dark green
            (0, 0, 128),    # Dark red
        ]
        
        # Use modulo to handle more classes than colors
        return colors[class_id % len(colors)]
    
    # ---------- Callback Functions ----------

    def image_callback(self, ros_image):
        """
        Process incoming images and run segmentation.

        Args:
            ros_image: ROS Image message
        """
        try:
            # Check if we should process this image (rate limiting)
            if not self.should_process_image():
                return
                
            # Update last process time
            self.last_process_time = rospy.Time.now()
            
            # Convert ROS Image to OpenCV format
            cv_image = self.convert_to_cv_image(ros_image)
            
            # Run YOLO inference
            results = self.run_yolo_inference(cv_image)
            
            # Create and publish structured data (always required)
            # Create and publish bounding box data in structured format
            bbox_array = self.create_bbox_array(results, ros_image.header)
            self.bbox_data_pub.publish(bbox_array)
            
            # Create and publish mask data in structured format
            mask_array = self.create_mask_array(cv_image, results)
            self.mask_data_pub.publish(mask_array)
            
            # Create and publish visualization topics (optional)
            if self.publish_visualizations:
                # Annotate the image with detection results
                annotated_image = self.annotate_detections(cv_image, results)
                
                # Create separate mask and bounding box visualizations
                mask_image = self.create_masks_image(cv_image, results)
                boxes_image = self.create_boxes_image(cv_image, results)
                
                # Publish mask image for visualization
                mask_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding='bgr8')
                mask_msg.header = ros_image.header
                self.masks_pub.publish(mask_msg)
                
                # Publish box image for visualization
                boxes_msg = self.bridge.cv2_to_imgmsg(boxes_image, encoding='bgr8')
                boxes_msg.header = ros_image.header
                self.boxes_pub.publish(boxes_msg)
                
                # Convert back to ROS Image and publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                annotated_msg.header = ros_image.header  # Preserve header for synchronization
                self.image_pub.publish(annotated_msg)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

def main():
    try:
        node = Yolo_Segmentation_Node()
        node.enable_gpu_acceleration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()