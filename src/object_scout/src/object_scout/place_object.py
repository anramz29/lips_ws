import rospy
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import sensor_msgs.msg
import tf2_ros
from tf.transformations import quaternion_matrix
import numpy as np
from cv_bridge import CvBridge

class PlaceObject:
    def __init__(self,robot_name, bbox_depth_topic,
                 camera_info_topic, depth_topic, init_node=False):
        
        if init_node:
            rospy.init_node('place_object', anonymous=True)

        self.robot_name = robot_name
        self.bbox_depth_topic = bbox_depth_topic
        self.camera_info_topic = camera_info_topic
        self.depth_topic = depth_topic
        
        self.camera_info = None
        self.latest_depth = None
        self.latest_bbox = None
        
        self.bridge = CvBridge()  # Initialize CvBridge

        self.bot = InterbotixLocobotXS(
            "locobot_wx250s", 
            arm_model="mobile_wx250s",
            init_node=False
        )

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)  # Store listener in self

        self.bbox_corners = None
        self.bbox_depth = None

    def setup_ros_communication(self):
        """
        Set up ROS communication for placing objects.
        """
        # Switch from keypoint subscription to bbox subscription

        self.bbox_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.bbox_callback
        )
        
        # Get camera info
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic,
            sensor_msgs.msg.CameraInfo,
            self.camera_info_callback
        )

        # Get depth images
        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            sensor_msgs.msg.Image,
            self.depth_callback
        )

    def depth_callback(self, msg):
        """Store the latest depth image."""
        self.latest_depth = msg

    def bbox_callback(self, msg):
        """
        Callback function for bounding box messages
        
        Args:
            msg (Float32MultiArray): Message containing bounding box coordinates
                Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
        """
        try:
            # Extract values from new format
            n_boxes = int(msg.data[0])
            if n_boxes < 1:
                self.bbox_corners = None
                rospy.logwarn("Place Object: No bounding boxes detected")
                return
                
            # Get coordinates from the first detected box
            # We'll use this similar to the FineApproacher implementation
            x1 = float(msg.data[3])
            y1 = float(msg.data[4])
            x2 = float(msg.data[5])
            y2 = float(msg.data[6])
            
            # Store the bounding box corners
            self.bbox_corners = {
                'x_min': x1,
                'y_min': y1,
                'x_max': x2,
                'y_max': y2
            }
            
            # Get depth information if available
            if len(msg.data) > 7:
                self.bbox_depth = float(msg.data[7])
            else:
                self.bbox_depth = None
                
            rospy.loginfo(f"Received bounding box: x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}")
            
        except Exception as e:
            rospy.logerr(f"Error parsing bounding box message: {e}")
            self.bbox_corners = None

    def camera_info_callback(self, msg):
        """Callback function for camera info messages.

        Args:
            msg (sensor_msgs.msg.CameraInfo): The message containing camera info data.
        """
        self.camera_info = msg
        rospy.loginfo(f"Received camera info: {msg}")

    def get_camera_info(self):
        """Get the focal length and principal point from the camera info.

        Returns:
            tuple: A tuple containing the focal length (fx, fy) and principal point (cx, cy).
        """
        if self.camera_info is None:
            rospy.logerr("Camera info not available")
            return (0, 0, 0, 0)
        
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        
        return (fx, fy, cx, cy)

    def convert_depth_image_to_cv2(self, depth_msg):
        """
        Convert ROS depth image to OpenCV format
        
        Args:
            depth_msg (sensor_msgs.msg.Image): ROS depth image message
            
        Returns:
            numpy.ndarray: OpenCV depth image (float32)
        """
        try:
            # Convert to CV image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            depth_image = depth_image.astype(np.float32) / 1000.0  # Convert from mm to meters
            
            
            return depth_image
            
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
            return np.zeros((480, 640), dtype=np.float32)  # Return empty image as fallback

    def calculate_bbox_center(self):
        """
        Calculate the center of the bounding box
        
        Returns:
            tuple: (x_center, y_center) coordinates, or (0, 0) if no bbox
        """
        if not self.bbox_corners:
            rospy.logwarn("No bounding box detected")
            return (0, 0)
        
        x_center = (self.bbox_corners['x_min'] + self.bbox_corners['x_max']) / 2
        y_center = (self.bbox_corners['y_min'] + self.bbox_corners['y_max']) / 2
        
        rospy.loginfo(f"Calculated bbox center at ({x_center:.1f}, {y_center:.1f})")
        return (x_center, y_center)

    def get_camera_to_base_transform(self):
        """
        Get the transformation matrix from camera frame to robot base frame.
        
        Returns:
            list: 4x4 transformation matrix
        """
        try:
            # Use TF to get the transfor
            
            # Wait for the transform to be available
            transform = self.buffer.lookup_transform(
                'base_link',  # Target frame (robot base)
                'camera_color_optical_frame',  # Source frame (camera)
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            # Extract translation
            translation = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            # Extract rotation (quaternion)
            rotation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            # Create transformation matrix
            rot_matrix = quaternion_matrix(rotation)
            rot_matrix[0][3] = translation[0]
            rot_matrix[1][3] = translation[1]
            rot_matrix[2][3] = translation[2]
            
            return rot_matrix
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return None
    
    def image_to_world_coordinates(self, x, y):
        """Convert image coordinates to world coordinates using the camera info.

        Args:
            x (float): The x coordinate in the image.
            y (float): The y coordinate in the image.

        Returns:
            tuple: A tuple containing the world coordinates (X, Y, Z).
        """
        if self.camera_info is None or self.latest_depth is None:
            rospy.logerr("Camera info or depth image not available")
            return (0, 0, 0)
        
        fx, fy, cx, cy = self.get_camera_info()
        
        # Get the depth value at the pixel (x, y)
        depth_image_cv = self.convert_depth_image_to_cv2(self.latest_depth)
        
        # Make sure coordinates are within image bounds
        h, w = depth_image_cv.shape
        x_px, y_px = min(max(0, int(x)), w-1), min(max(0, int(y)), h-1)
        
        depth_value = depth_image_cv[y_px, x_px]
        
        # Check if depth is valid
        if depth_value == 0 or np.isnan(depth_value):
            rospy.logwarn(f"Invalid depth at ({x}, {y}): {depth_value}")
            return None
        
        # Convert to camera coordinates
        X = (x - cx) * depth_value / fx
        Y = (y - cy) * depth_value / fy
        Z = depth_value
        
        # Transform to robot base coordinates
        camera_to_base = self.get_camera_to_base_transform()
        if camera_to_base is None:
            return None
            
        # Apply transformation (camera coords to base coords)
        point_camera = np.array([X, Y, Z, 1.0])
        point_base = np.dot(camera_to_base, point_camera)
        
        return (point_base[0], point_base[1], point_base[2])

    def place_at_bbox_center(self):
        """
        Move the robot arm to place at the center of the detected bounding box.
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Give the detection system time to find the box
        rospy.sleep(2.0)
        
        # Get the center of the bounding box
        image_center_x, image_center_y = self.calculate_bbox_center()
        
        if image_center_x == 0 and image_center_y == 0:
            rospy.logwarn("No bounding box detected, cannot place object")
            return False
        
        # Convert to world coordinates
        world_coords = self.image_to_world_coordinates(image_center_x, image_center_y)
        
        if world_coords is None:
            rospy.logerr("Failed to convert to world coordinates")
            return False
        
        world_x, world_y, world_z = world_coords
        
        rospy.loginfo(f"Target position in robot coordinates: ({world_x}, {world_y}, {world_z})")
        
        # Add some offset for placing (e.g., slightly above the target)
        place_z = max(world_z + 0.05, 0.15)  # 5cm above the detected point, minimum 15cm
        
        # Use the Interbotix API to move the arm
        try:
            # First, move above the target
            self.bot.arm.set_ee_pose_components(x=world_x, y=world_y, z=place_z)
            rospy.sleep(1.0)  # Wait for motion to complete
            
            # Move down to place the object
            lower_z = max(world_z + 0.01, 0.05)  # 1cm above detected surface, minimum 5cm
            self.bot.arm.set_ee_pose_components(x=world_x, y=world_y, z=lower_z)
            rospy.sleep(1.0)
            
            # Open gripper to release object
            self.bot.gripper.open()
            rospy.sleep(1.0)
            
            # Move back up before going home
            self.bot.arm.set_ee_pose_components(x=world_x, y=world_y, z=place_z)
            rospy.sleep(1.0)
            
            # Return to home position
            self.bot.arm.go_to_sleep_pose()
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to execute place motion: {e}")
            
            # Ensure arm returns to a safe position
            self.bot.arm.go_to_sleep_pose()
            return False


