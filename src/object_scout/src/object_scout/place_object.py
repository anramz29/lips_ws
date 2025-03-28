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
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        self.camera_info_topic = f'/{self.robot_name}/camera/color/camera_info'
        self.keypoint_topic = f'/{self.robot_name}/camera/yolo/keypoints'  # Fixed slash consistency
        self.depth_topic = f'/{self.robot_name}/camera/depth/image_rect_raw'

        self.keypoints = []
        self.camera_info = None
        self.latest_depth = None
        
        self.bridge = CvBridge()  # Initialize CvBridge

        self.bot = InterbotixLocobotXS(
            "locobot_wx250s", 
            arm_model="mobile_wx250s",
            init_node=False
        )

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)  # Store listener in self

    def setup_ros_communication(self):
        """
        Set up ROS communication for placing objects.
        """
        # Fix subscriber setup
        self.keypoint_sub = rospy.Subscriber(
            self.keypoint_topic,
            Float32MultiArray,
            self.keypoint_callback
        )
        
        # This shouldn't use wait_for_message with a callback
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic,
            sensor_msgs.msg.CameraInfo,
            self.camera_info_callback
        )

        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            sensor_msgs.msg.Image,
            self.depth_callback
        )

    def depth_callback(self, msg):
        """Store the latest depth image."""
        self.latest_depth = msg


    def keypoint_callback(self, msg):
        """Callback function for keypoint detection messages.

        Args:
            msg (Float32MultiArray): The message containing keypoint data.
        """
        # extract indexes after 3rd element 
        self.keypoints = list(msg.data[3:]) if len(msg.data) > 3 else []


        rospy.loginfo(f"Received keypoints: {self.keypoints}")

    def enable_keypoint_detection(self, enable=True):
        """Enable or disable keypoint detection by calling the appropriate service.

        Args: 
            enable (bool): True to enable keypoint detection, False to disable it.

        Returns:
            bool: True if the service call was successful, False otherwise.
        """

        service_name = f'/{self.robot_name}/keypoint_detector/set_enabled'
        
        try:
            # Wait for service to be available
            rospy.wait_for_service(service_name, timeout=3.0)
            
            # Call the service
            set_enabled = rospy.ServiceProxy(service_name, SetBool)
            response = set_enabled(enable)
            
            if response.success:
                state = "enabled" if enable else "disabled"
                rospy.loginfo(f"Keypoint detection {state}")
                return True
            else:
                rospy.logerr(f"Failed to set keypoint detection: {response.message}")
                return False
                
        except rospy.ROSException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
 
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
    
    def get_centeroid(self):
        """Calculate the centroid of the detected keypoints.

        Returns:
            tuple: A tuple containing the x and y coordinates of the centroid.
        """
        if not self.keypoints:
            return (0, 0)
        
        # Calculate the average of the keypoints
        x_coords = [self.keypoints[i] for i in range(0, len(self.keypoints), 2)]
        y_coords = [self.keypoints[i] for i in range(1, len(self.keypoints), 2)]
        
        centroid_x = sum(x_coords) / len(x_coords)
        centroid_y = sum(y_coords) / len(y_coords)
        
        return (centroid_x, centroid_y)
    
    def convert_depth_image_to_cv2(self, ros_image):
        """Convert ROS depth image to meters in OpenCV format."""
        try:
            depth = self.bridge.imgmsg_to_cv2(ros_image, '16UC1')
            depth = depth.astype(float) / 1000.0  # Convert from mm to meters
            return depth
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {str(e)}")
            return np.zeros((480, 640), dtype=np.float32)
    


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
    
    def place_at_centroid(self):
        """
        Move the robot arm to place at the centroid of detected keypoints.
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Get the centroid in image coordinates
        image_centroid_x, image_centroid_y = self.get_centeroid()
        
        if image_centroid_x == 0 and image_centroid_y == 0:
            rospy.logwarn("No keypoints detected, cannot place object")
            return False
        
        # Convert to world coordinates
        world_coords = self.image_to_world_coordinates(image_centroid_x, image_centroid_y)
        
        if world_coords is None:
            rospy.logerr("Failed to convert to world coordinates")
            return False
        
        world_x, world_y, world_z = world_coords
        
        rospy.loginfo(f"Target position in robot coordinates: ({world_x}, {world_y}, {world_z})")
        
        # Add some offset for placing (e.g., slightly above the target)
        place_z = 0.15  # 5cm above the detected point
        
        # Use the Interbotix API to move the arm
        try:
            # First, move above the target
            self.bot.arm.set_ee_pose_components(x=world_x, y=world_y, z=place_z)
            rospy.sleep(1.0)  # Wait for motion to complete
            
            # Open gripper to release object
            self.bot.gripper.open()
            rospy.sleep(1.0)
            
            # Return to home position
            self.bot.arm.go_to_sleep_pose()
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to execute place motion: {e}")
            self.bot.arm.go_to_sleep_pose()  # Ensure arm returns to a safe position
            return False
        
    