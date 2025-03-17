#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray  

class SegmentationDistanceNode:
    """
    Node for calculating the depth of objects detected by YOLO segmentation.
    Subscribes to polygon-based segmentation masks and depth images, and calculates 
    average depth within each segmented region.
    """
    def __init__(self):
        rospy.init_node('segmentation_distance_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Load parameters from parameter server
        # Published topics
        self.mask_viz_depth_topic = rospy.get_param('~depth_visualization_topic')
        self.distance_topic = rospy.get_param('~distance_topic', 'segmentation_distances')
        
        # Subscribed topics
        self.depth_image_topic = rospy.get_param('~depth_image_topic')
        self.mask_data_topic = rospy.get_param('~mask_data_topic')
        self.mask_viz_topic = rospy.get_param('~masks_topic')

        # Debug mode
        self.debug = rospy.get_param('~debug_mode', False)
        
        # Message storage
        self.last_mask_msg = None
        self.last_depth_msg = None
        self.last_mask_viz_msg = None
        
        # Metadata storage
        self.detection_metadata = []  # Store class_ids, scores, etc.
        
        # Image dimensions - will be set when we receive the depth image
        self.image_height = None
        self.image_width = None
        
        rospy.loginfo(f"Segmentation Distance Node initialized. Using mask topic: {self.mask_data_topic}")

        # Setup ROS communication
        self._setup_ros_communication()
        rospy.loginfo("ROS communication setup complete.")

    # ---------- ROS Communication Setup ----------

    def _setup_ros_communication(self):

        # Subscribers
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        self.mask_sub = rospy.Subscriber(self.mask_data_topic, Float32MultiArray, self.mask_callback)
        self.mask_viz_sub = rospy.Subscriber(self.mask_viz_topic, Image, self.mask_viz_callback)

        # Publishers
        self.mask_viz_depth_pub = rospy.Publisher(self.mask_viz_depth_topic, Image, queue_size=10)
        self.distance_pub = rospy.Publisher(self.distance_topic, Float32MultiArray, queue_size=10)
        
        
        rospy.loginfo(f"Subscribed to depth topic: {self.depth_image_topic}")
        rospy.loginfo(f"Subscribed to mask topic: {self.mask_data_topic}")

   
    # ---------- Callbacks ----------

    def depth_callback(self, msg):
        self.last_depth_msg = msg
        self.image_height = msg.height
        self.image_width = msg.width

    def mask_viz_callback(self, msg):
        # obtain the image of the mask
        self.last_mask_viz_msg = msg

    def mask_callback(self, msg):
        """
        Process incoming mask data and trigger processing with depth data if available
        """

        self.last_mask_msg = msg
        # Process masks and depth data
        self.process_masks()

    def parse_mask_array(self, mask_array):
        """
        Parse incoming mask data and convert to a simple dictionary format.
        
        Args:
            mask_array: Float32MultiArray message with mask data
            
        Returns:
            list: List of dictionaries containing parsed mask data
                Each dictionary has:
                - class_id: The object class ID (int)
                - confidence: Detection confidence (float)
                - contour: List of (x,y) tuples representing the contour points
        """
        # Get the data array
        data = mask_array.data
        
        # First element is the number of objects
        num_objects = int(data[0])
        
        # Prepare the result list
        parsed_objects = []
        
        # Return empty list if no objects
        if num_objects <= 0:
            return parsed_objects
        
        # Parse each object's data
        index = 1  # Start after the num_objects element
        
        for _ in range(num_objects):
            # Extract object metadata
            class_id = int(data[index])
            confidence = float(data[index + 1])
            num_points = int(data[index + 2])
            
            index += 3  # Move to first point
            
            # Extract points
            points = []
            for i in range(num_points):
                if index + i*2 + 1 < len(data):  # Ensure we don't go out of bounds
                    x = int(data[index + i*2])
                    y = int(data[index + i*2 + 1])
                    points.append((x, y))
            
            index += num_points * 2  # Move to separator or next object
            
            # Skip the separator (-1, -1) if present
            if index + 1 < len(data) and data[index] == -1.0 and data[index + 1] == -1.0:
                index += 2
            
            # Add to results
            parsed_objects.append({
                'class_id': class_id,
                'confidence': confidence,
                'contour': points
            })
        
        return parsed_objects


    def _check_data_available(self):
        """
        Check if necessary data is available for processing.
        
        Returns:
            bool: True if both mask and depth data are available
        """
        if self.last_mask_msg is None or self.last_depth_msg is None:
            rospy.logwarn("Missing mask or depth data. Skipping processing.")
            return False
        return True

    def _calculate_object_depths(self, parsed_objects, depth_image):
        """
        Calculate average depth for each object using its contour mask.
        
        Args:
            parsed_objects (list): List of parsed object dictionaries
            depth_image (np.ndarray): Depth image as numpy array
            
        Returns:
            list: List of dictionaries with object data including average depth
        """
        average_depths = []
        
        for obj in parsed_objects:
            # Create mask from contour
            mask = self._create_mask_from_contour(obj['contour'])
            
            # Calculate average depth within mask
            avg_depth = self._calculate_average_depth(depth_image, mask)
            
            # Add to results
            average_depths.append({
                'class_id': obj['class_id'],
                'confidence': obj['confidence'],
                'avg_depth': avg_depth
            })
        
        return average_depths

    def _create_mask_from_contour(self, contour):
        """
        Create a binary mask from contour points.
        
        Args:
            contour (list): List of (x,y) tuples representing contour points
            
        Returns:
            np.ndarray: Binary mask
        """
        mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
        contour_array = np.array(contour, dtype=np.int32)
        cv2.fillPoly(mask, [contour_array], 255)
        return mask

    def _calculate_average_depth(self, depth_image, mask):
        """
        Calculate average depth within masked region.
        Convert from millimeters to meters.
        
        Args:
            depth_image (np.ndarray): Depth image
            mask (np.ndarray): Binary mask
            
        Returns:
            float: Average depth value in meters
        """
        # Apply mask to depth image
        masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=mask)
        
        # Get non-zero pixels (where mask is applied)
        non_zero_pixels = masked_depth[mask > 0]
        
        # Calculate average depth if we have valid depth values
        if len(non_zero_pixels) > 0:
            # Filter out zeros and possibly invalid depth values
            valid_depths = non_zero_pixels[non_zero_pixels > 0]
            if len(valid_depths) > 0:
                # Convert from mm to meters (depth data is typically in mm)
                return np.mean(valid_depths) / 1000.0
        
        return 0.0

    def _publish_depth_results(self, average_depths, parsed_objects):
        """
        Publish depth results as a Float32MultiArray.
        
        Args:
            average_depths (list): List of dictionaries with object data
            parsed_objects (list): List of parsed object dictionaries with contours
        """
        distance_array = Float32MultiArray()
        
        for i, (obj, depth_info) in enumerate(zip(parsed_objects, average_depths)):
            # Calculate centroid of the contour
            contour = np.array(obj['contour'], dtype=np.int32)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                # Fallback if moments calculation fails
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
            
            # Add class_id, confidence, depth, center_x, center_y
            distance_array.data.extend([
                float(obj['class_id']), 
                obj['confidence'], 
                depth_info['avg_depth'],
                float(cx),
                float(cy)
            ])
        
        self.distance_pub.publish(distance_array)

    def process_masks(self):
        """
        Process the last received mask data and depth data.
        """
        # Check if we have both mask and depth data
        if not self._check_data_available():
            return []

        # Parse the mask data
        parsed_objects = self.parse_mask_array(self.last_mask_msg)
        
        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(self.last_depth_msg, '16UC1')
        depth_image = depth_image.astype(float)
        
        # If no objects are detected, still publish the original visualization
        if not parsed_objects:
            if self.debug:
                rospy.loginfo("No objects detected in the mask data. Publishing original visualization.")
            
            # Check if we have a visualization image to publish
            if self.last_mask_viz_msg is not None:
                # Just republish the original mask visualization
                viz_image = self.bridge.imgmsg_to_cv2(self.last_mask_viz_msg, desired_encoding="bgr8")
                self.mask_viz_depth_pub.publish(self.last_mask_viz_msg)
            
            return []
        
        # Process depth for each object
        average_depths = self._calculate_object_depths(parsed_objects, depth_image)
        
        # Prepare and publish results
        self._publish_depth_results(average_depths, parsed_objects)
        
        # Create and publish visualization
        self._create_depth_visualization(parsed_objects, depth_image, average_depths)
        
        return average_depths

    def _create_depth_visualization(self, parsed_objects, depth_image, average_depths):
        """
        Create and publish visualization of depth values overlaid on masks.
        """
        
        if self.last_mask_viz_msg is None:
            rospy.logwarn("Missing mask visualization. Cannot create depth visualization.")
            return
        
        # Convert mask visualization to OpenCV format
        viz_image = self.bridge.imgmsg_to_cv2(self.last_mask_viz_msg, desired_encoding="bgr8")

        # Create depth visualization
        depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        # Use this instead if you just want to annotate the original visualization
        combined_viz = viz_image.copy()

        # Add depth values to visualization
        for i, (obj, depth_info) in enumerate(zip(parsed_objects, average_depths)):
            contour = np.array(obj['contour'], dtype=np.int32)
            
            # Get bounding box of contour
            x, y, w, h = cv2.boundingRect(contour)
            
            # Place depth text at the bottom of the bounding box
            text = f"{depth_info['avg_depth']:.2f}m"
            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # Position text at the bottom of the object
            text_x = x + (w - text_size[0]) // 2
            text_y = y + h + text_size[1] + 5
            
 
            
            # Draw depth value on the combined visualization
            cv2.putText(combined_viz, text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        

        self.mask_viz_depth_pub.publish(self.bridge.cv2_to_imgmsg(combined_viz, encoding="bgr8"))

   
    
def main():
    try:
        SegmentationDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()