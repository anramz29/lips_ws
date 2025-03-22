#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class YoloKeypointDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_keypoint_detection_node', anonymous=True)
        
        # Global parameters
        self.debug_mode = rospy.get_param('~debug_mode')
        self.enabled_by_default = rospy.get_param('~enabled_by_default')

        # YoLo model parameters
        self.model_path = rospy.get_param('~model')
        self.device = rospy.get_param('~device')
        self.input_size = rospy.get_param('~input_size')

        # input topics
        self.image_topic = rospy.get_param('~image_topic')
        self.camera_frame = rospy.get_param('~camera_frame')
        self.base_frame = rospy.get_param('~base_frame')
        self.image_depth_topic = rospy.get_param('~image_depth_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')

        # Output topics
        self.keypoints_topic = rospy.get_param('~keypoints_topic', 'camera/yolo/keypoints')
        self.keypoints_viz_topic = rospy.get_param('~keypoints_viz_topic', 'camera/yolo/keypoints_visualization')
        
        # Intialize Image processing bridge and tf2
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize model to None (lazy loading)
        self.model = None
        self.load_model_when_needed = True  
        
        # State variables
        self.latest_depth = None
        self.latest_image = None
        self.camera_info = None
        self.camera_dict = {}
        self.last_process_time = rospy.Time(0)
        self.process_interval = rospy.Duration(1.0 / 10)  # 10 Hz max

        # Initialize ROS communication
        self.setup_ros_communication()

        rospy.loginfo(f"Keypoint detection node initialized (initially disabled)")
        rospy.loginfo(f"Will use model: {self.model_path}")
        rospy.loginfo(f"Subscribing to: {self.image_topic}")
        rospy.loginfo(f"Publishing to: {self.keypoints_topic} and {self.keypoints_viz_topic}")

    
    # --------------- ROS Communication --------------- #
        
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""

        # Setup publishers
        self.keypoints_pub = rospy.Publisher(
            self.keypoints_topic, 
            Float32MultiArray, 
            queue_size=1
        )

        self.viz_pub = rospy.Publisher(
            self.keypoints_viz_topic, 
            Image, 
            queue_size=1
        )
        
        # Setup subscribers
        self.enable_sub = rospy.Subscriber(
            'keypoint_detector/enable',
            Bool,
            self.enable_callback
        )
        
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.depth_image_sub = rospy.Subscriber(
            self.image_depth_topic,
            Image,
            self.depth_image_callback,
            queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic,
            geometry_msgs.msg.CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

    # --------------- Callbacks --------------- #
    
    def enable_callback(self, msg):
        """Handle enable/disable messages"""
        if msg.data != self.enabled_by_default:
            self.enabled_by_default = msg.data
            state = "enabled" if self.enabled else "disabled"
            rospy.loginfo(f"Keypoint detection {state}")
            
            # Load model when first enabled
            if self.enabled_by_default and self.load_model_when_needed and self.model is None:
                rospy.loginfo("Loading YOLO keypoint model...")
                try:
                    self.model = YOLO(self.model_path)
                    rospy.loginfo("Model loaded successfully")
                    self.load_model_when_needed = False
                except Exception as e:
                    rospy.logerr(f"Failed to load model: {e}")
                    self.enabled = False
    
    def should_process_image(self):
        """Determine if we should process this image frame"""
        if not self.enabled_by_default:
            return False
            
        current_time = rospy.Time.now()
        if (current_time - self.last_process_time) < self.process_interval:
            return False
            
        self.last_process_time = current_time
        return True
    
    def depth_image_callback(self, msg):
        """
        Callback for depth image. Currently unused but can be extended for depth processing.
        
        Args:
            msg: ROS Image message for depth image
        """
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")
            self.latest_depth = None

    def camera_info_callback(self, msg):
        """
        Store camera information for later use.
        
        Args:
            msg: CameraInfo message containing camera parameters
        """
        self.camera_info = msg

        # Extract camera intrinsics
        fx = self.camera_info.K[0] 
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        # Store camera info for later use
        self.camera_dict = {
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy
        }

        rospy.loginfo("Camera info received")
        rospy.loginfo(f"Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        
    # --------------- Coordinate Transformation --------------- #

    def transform_to_base_frame(self, point_camera):
        """
        Transform a 3D point from camera frame to base frame.
        
        Args:
            point_camera (numpy.ndarray): 3D point in camera frame [x, y, z]
                
        Returns:
            numpy.ndarray: Transformed 3D point in base frame [x', y', z']
        """
        
        try:
            # Create a PointStamped message
            point_stamped = geometry_msgs.msg.PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # Transform the point
            point_transformed = self.tf_buffer.transform(point_stamped, self.base_frame)
            
            # Return as numpy array
            return np.array([
                point_transformed.point.x,
                point_transformed.point.y,
                point_transformed.point.z
            ])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {str(e)}")
            return None
       
        
    # --------------- Image Proccesing --------------- #

    def run_model(self, image):
        """
        Run the YOLO model on the given image and return detected keypoints.

        Args:
            image (numpy.ndarray): The input image in BGR format.

        Returns:
            list: A list of detected keypoints, where each keypoint is represented as a numpy array.
        """

        if self.model is None:
            rospy.logwarn("Model not loaded, skipping detection")
            return None
        
        results = self.model.predict(
            image, 
            device=self.device, 
            imgsz=self.input_size
        )

        keypoints = []
        
        for result in results:
            if result.keypoints is not None:
                keypoints.append(result.keypoints[0].cpu().numpy())
        
        return keypoints
    
    def pixel_to_3d(self, u, v, depth):
        """
        Convert pixel coordinates and depth to 3D camera coordinates.
        
        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate
            depth: Depth in meters
            
        Returns:
            Point: 3D point in camera coordinates or None if conversion fails
        """
        if self.camera_info is None:
            rospy.logwarn("No camera info received yet")
            return None
        

        fx = self.camera_dict['fx']
        fy = self.camera_dict['fy']
        cx = self.camera_dict['cx']
        cy = self.camera_dict['cy']

        # Check if depth is valid
        if depth <= 0:
            rospy.logwarn("Invalid depth value")
            return None
        
        # Convert pixel coordinates to camera coordinates
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return np.array([x, y, z])  # Return as a numpy array for consistency

    def calculate_approach_vector(self, keypoints):
        """
        Calculate approach vector based on keypoints.
        
        Args:
            keypoints: List of keypoints from YOLO model
            
        Returns:
            tuple: (target_position, approach_direction) in base frame
        """
        if not keypoints or len(keypoints) == 0:
            rospy.logwarn("No keypoints detected")
            return None, None
        
        # Extract valid keypoints (with confidence > threshold)
        valid_keypoints = []
        for kp_set in keypoints:
            for kp in kp_set:
                if kp[2] > 0.5:  # Confidence threshold
                    valid_keypoints.append(kp)
        
        if not valid_keypoints:
            rospy.logwarn("No valid keypoints with sufficient confidence")
            return None, None
        
        # Calculate centroid of keypoints
        keypoint_coords = np.array([kp[:2] for kp in valid_keypoints])
        centroid = np.mean(keypoint_coords, axis=0)
        
        # Get depth at centroid position
        if self.latest_depth is None:
            rospy.logwarn("No depth image available")
            return None, None
        
        # Convert centroid to integer pixel coordinates
        u, v = int(centroid[0]), int(centroid[1])
        
        # Ensure coordinates are within image bounds
        if (u < 0 or u >= self.latest_depth.shape[1] or 
            v < 0 or v >= self.latest_depth.shape[0]):
            rospy.logwarn(f"Centroid outside image bounds: ({u}, {v})")
            return None, None
        
        # Get depth value (might need scaling depending on your depth image format)
        depth_value = self.latest_depth[v, u]
        if depth_value == 0:  # Often 0 indicates invalid depth
            rospy.logwarn("Invalid depth at keypoint centroid")
            return None, None
            
        # Convert from raw depth value to meters (adjust scale factor as needed)
        depth_meters = depth_value / 1000.0  # Example: convert mm to meters
        
        # Convert 2D centroid + depth to 3D point in camera frame
        target_point_camera = self.pixel_to_3d(u, v, depth_meters)
        if target_point_camera is None:
            return None, None
        
        # Transform point to base frame
        target_point_base = self.transform_to_base_frame(target_point_camera)
        if target_point_base is None:
            return None, None
        
        # Calculate approach direction (from base to target)
        # Assuming robot base is at origin of base frame
        approach_direction = target_point_base / np.linalg.norm(target_point_base)
        
        return target_point_base, approach_direction
    
    def visualize_keypoints(self, keypoints, target_position, approach_direction):
        """
        Create visualization of keypoints and approach vector.
        
        Args:
            keypoints: Detected keypoints
            target_position: Target position in base frame
            approach_direction: Approach direction vector
        """
        if self.latest_image is None:
            return
            
        viz_image = self.latest_image.copy()
        
        # Draw keypoints
        for kp_set in keypoints:
            for kp in kp_set:
                if kp[2] > 0.5:  # Confidence threshold
                    x, y = int(kp[0]), int(kp[1])
                    cv2.circle(viz_image, (x, y), 5, (0, 255, 0), -1)
        
        # Draw centroid
        keypoint_coords = []
        for kp_set in keypoints:
            for kp in kp_set:
                if kp[2] > 0.5:
                    keypoint_coords.append(kp[:2])
                    
        if keypoint_coords:
            centroid = np.mean(np.array(keypoint_coords), axis=0)
            cv2.circle(viz_image, (int(centroid[0]), int(centroid[1])), 8, (255, 0, 0), -1)
        
        # Add text with distance information
        distance = np.linalg.norm(target_position)
        cv2.putText(
            viz_image,
            f"Distance: {distance:.2f}m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )
        
        # Add text with approach direction
        cv2.putText(
            viz_image,
            f"Dir: [{approach_direction[0]:.2f}, {approach_direction[1]:.2f}, {approach_direction[2]:.2f}]",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        
        # Publish visualization
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding="bgr8")
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            rospy.logerr(f"Failed to publish visualization: {e}")

    # -------------- Main Function --------------- #

    def image_callback(self, msg):
        """
        Callback for processing incoming images.
        
        Args:
            msg: ROS Image message

        Publsher Output Format:
            keypoints_msg: Float32MultiArray containing target position and approach direction
        """
        if not self.should_process_image():
            return
        
        # Convert ROS Image to OpenCV image
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return
        
        # Check if we have all required data
        if self.latest_depth is None:
            rospy.logwarn_throttle(1.0, "No depth image available yet")
            return
            
        if self.camera_info is None:
            rospy.logwarn_throttle(1.0, "No camera info available yet")
            return
        
        # Detect keypoints using YOLO
        keypoints = self.run_model(self.latest_image)
        if not keypoints:
            return
        
        # Calculate approach vector
        target_position, approach_direction = self.calculate_approach_vector(keypoints)
        if target_position is None or approach_direction is None:
            return
        
        # Prepare message for publishing
        keypoints_msg = Float32MultiArray()
        keypoints_msg.layout.dim = [
            MultiArrayDimension(label="vectors", size=6, stride=6)
        ]
        
        # Concatenate target position and approach direction
        keypoints_msg.data = np.concatenate([target_position, approach_direction]).tolist()
        
        # Publish keypoints
        self.keypoints_pub.publish(keypoints_msg)
        
        # If in debug mode, visualize the results
        if self.debug_mode:
            self.visualize_keypoints(keypoints, target_position, approach_direction)


if __name__ == '__main__':
    try:
        node = YoloKeypointDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass