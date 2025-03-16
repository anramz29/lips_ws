#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

class SegmentationDistanceNode:
    """
    Node for calculating the depth of objects detected by YOLO segmentation.
    Subscribes to segmentation masks and depth images, and calculates 
    average depth within each segmented region.
    """
    def __init__(self):
        rospy.init_node('segmentation_distance_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters from parameter server
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.mask_data_topic = rospy.get_param('~mask_data_topic')
        self.visualization_topic = rospy.get_param('~visualization_topic')
        self.distance_topic = rospy.get_param('~distance_topic', 'segmentation_distances')
        
        # Message storage
        self.last_mask_msg = None
        self.last_depth_msg = None
        
        # Metadata storage
        self.detection_metadata = []  # Store class_ids, scores, etc.
        
        rospy.loginfo(f"Segmentation Distance Node initialized. Using mask topic: {self.mask_data_topic}")

        # Setup ROS communication
        self._setup_ros_communcation()
        rospy.loginfo("ROS communication setup complete.")

    # ---------- ROS Communication Setup ----------

    def _setup_ros_communcation(self):
        # Publishers
        self.viz_pub = rospy.Publisher(self.visualization_topic, Image, queue_size=10)
        self.distance_pub = rospy.Publisher(self.distance_topic, Float32MultiArray, queue_size=10)
        
        # Subscribers
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        self.mask_sub = rospy.Subscriber(self.mask_data_topic, UInt8MultiArray, self.mask_callback)
        

    # ---------- Callback Functions ----------

    def mask_callback(self, msg):
        """Callback for segmentation mask data."""
        try:
            self.last_mask_msg = msg
            self.extract_metadata_from_mask(msg)
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in mask callback: {str(e)}")

    def depth_callback(self, msg):
        """Callback for depth images."""
        try:
            self.last_depth_msg = msg
            self.process_images()
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {str(e)}")

    def extract_metadata_from_mask(self, msg):
        """Extract class IDs and confidence scores from mask message."""
        # The mask message structure follows our defined format:
        # - layout.data_offset contains the size of metadata
        # - metadata format is [class_id1, conf1, class_id2, conf2, ...]
        try:
            metadata_size = msg.layout.data_offset
            if metadata_size > 0:
                # Extract metadata (first part of the data array)
                metadata_array = np.array(msg.data[:metadata_size], dtype=np.uint8)
                
                # Reshape into pairs
                self.detection_metadata = []
                for i in range(0, len(metadata_array), 2):
                    if i+1 < len(metadata_array):
                        class_id = int(metadata_array[i])
                        confidence = float(metadata_array[i+1]) / 255.0  # Normalize if needed
                        self.detection_metadata.append((class_id, confidence))
        except Exception as e:
            rospy.logerr(f"Error extracting metadata from mask: {str(e)}")
            self.detection_metadata = []

    # ---------- Main Processing Function ----------

    def process_images(self):
        """Process the latest mask and depth images to extract depth for segmented objects."""
        # Skip if we don't have both messages
        if self.last_mask_msg is None or self.last_depth_msg is None:
            return

        try:
            # Convert depth image to OpenCV format
            depth = self.convert_depth_image_to_cv2()
            
            # Create a visualization image from the depth image
            viz_img = self.create_visualization_image(depth)
            
            # Process mask depth
            self.process_mask_depth(depth, viz_img)
            
            # Publish visualization
            self.publish_visualization(viz_img)
            
        except Exception as e:
            rospy.logerr(f"Error in process_images: {str(e)}")

    # ---------- Image Conversion Functions ----------

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

    def create_visualization_image(self, depth):
        """Create a color visualization image from depth data."""
        # Normalize depth for visualization (0-255)
        norm_depth = depth.copy()
        norm_depth[norm_depth == 0] = np.nan  # Set zeros to NaN
        valid_mask = ~np.isnan(norm_depth)
        
        if np.any(valid_mask):
            min_val = np.nanmin(norm_depth)
            max_val = np.nanmax(norm_depth)
            
            # Scale to 0-255 range
            norm_depth = 255 * (norm_depth - min_val) / (max_val - min_val)
            norm_depth = np.nan_to_num(norm_depth, nan=0)
            norm_depth = norm_depth.astype(np.uint8)
        else:
            # If no valid depths, just use zeros
            norm_depth = np.zeros_like(depth, dtype=np.uint8)
        
        # Convert to color image (depth visualization)
        depth_color = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
        
        return depth_color

    # ---------- Processing Functions ----------

    def process_mask_depth(self, depth, viz_img):
        """
        Calculate and annotate the average depth for each segmentation mask.
        
        Args:
            depth: OpenCV depth image in meters
            viz_img: Visualization image to annotate
        """
        msg = self.last_mask_msg
        if msg.layout.dim and len(msg.layout.dim) >= 3:
            try:
                # Get dimensions from message
                n_objects = msg.layout.dim[0].size
                height = msg.layout.dim[1].size
                width = msg.layout.dim[2].size
                
                # Skip if no objects
                if n_objects == 0:
                    return
                
                # Get the start of the mask data (after metadata)
                metadata_size = msg.layout.data_offset
                mask_data = np.array(msg.data[metadata_size:], dtype=np.uint8)
                
                # Check if we have enough data
                expected_size = n_objects * height * width
                if len(mask_data) < expected_size:
                    rospy.logwarn(f"Mask data size mismatch: got {len(mask_data)}, expected {expected_size}")
                    return
                
                # Reshape masks: [objects, height, width]
                masks = np.reshape(mask_data, (n_objects, height, width))
                
                # Process each mask
                distances = []
                
                for i in range(n_objects):
                    # Convert mask to binary (255 -> 1)
                    binary_mask = (masks[i] > 0).astype(np.uint8)
                    
                    # Skip empty masks
                    if np.sum(binary_mask) == 0:
                        continue
                    
                    # Find contours for visualization
                    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    # Calculate center of mass for the mask
                    M = cv2.moments(binary_mask)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        # Fallback to geometric center if no moments
                        y, x = np.where(binary_mask > 0)
                        if len(x) > 0 and len(y) > 0:
                            cx, cy = int(np.mean(x)), int(np.mean(y))
                        else:
                            continue
                    
                    # Get class information if available
                    class_id, confidence = self.detection_metadata[i] if i < len(self.detection_metadata) else (-1, 0)
                    
                    # Extract depth values within mask
                    masked_depth = depth.copy()
                    masked_depth[binary_mask == 0] = 0  # Zero out areas outside mask
                    
                    # Calculate average of non-zero depth values
                    valid_depths = masked_depth[masked_depth > 0]
                    if len(valid_depths) > 0:
                        avg_depth = np.mean(valid_depths)
                        median_depth = np.median(valid_depths)
                        
                        # Calculate min, max depths and percentage of valid pixels
                        min_depth = np.min(valid_depths)
                        max_depth = np.max(valid_depths)
                        valid_percentage = len(valid_depths) / np.sum(binary_mask) * 100
                        
                        # For small objects, median may be more reliable than mean
                        if np.sum(binary_mask) < 100:  # If object is small (fewer than 100 pixels)
                            primary_depth = median_depth
                        else:
                            primary_depth = avg_depth
                        
                        # Store distance information 
                        # [object_id, class_id, confidence, primary_depth, avg_depth, median_depth, min_depth, max_depth, valid_percentage]
                        distances.append([
                            i, class_id, confidence, 
                            primary_depth, avg_depth, median_depth, 
                            min_depth, max_depth, valid_percentage
                        ])
                        
                        # Draw mask contour on visualization image
                        cv2.drawContours(viz_img, contours, -1, (0, 255, 0), 2)
                        
                        # Add distance text near object center
                        self.annotate_distance(viz_img, cx, cy, primary_depth, class_id)
                
                # Publish distance information
                if distances:
                    self.publish_distance_data(distances)
            
            except Exception as e:
                rospy.logerr(f"Error processing mask depth: {str(e)}")

    # ---------- Annotation and Publishing ----------

    def annotate_distance(self, img, x, y, depth, class_id):
        """Add depth text to the image at the specified position."""
        # Format text with class and depth
        if class_id >= 0:
            text = f"ID:{class_id} {depth:.2f}m"
        else:
            text = f"{depth:.2f}m"
            
        # Get text size
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        
        # Calculate text position
        text_x = max(0, x - text_size[0]//2)
        text_y = max(30, y)  # Ensure text is visible
        
        # Draw background rectangle
        cv2.rectangle(img, 
                     (text_x-5, text_y-text_size[1]-5), 
                     (text_x+text_size[0]+5, text_y+5), 
                     (0, 0, 0), -1)
        
        # Draw text
        cv2.putText(img, text, (text_x, text_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def publish_distance_data(self, distances):
        """
        Publish distance information for all detected objects.
        
        Format: [object_id, class_id, confidence, primary_depth, avg_depth, median_depth, min_depth, max_depth, valid_percentage, ...]
        """
        msg = Float32MultiArray()
        # Flatten the distances list
        msg.data = [float(val) for sublist in distances for val in sublist]
        self.distance_pub.publish(msg)

    def publish_visualization(self, img):
        """Publish the annotated image for visualization."""
        viz_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        viz_msg.header = self.last_depth_msg.header
        self.viz_pub.publish(viz_msg)


if __name__ == '__main__':
    try:
        node = SegmentationDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass