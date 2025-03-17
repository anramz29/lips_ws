import math
import time
import rospy
from visualization_msgs.msg import Marker
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.arm import InterbotixArmXSInterface
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
import numpy as np
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg

# Add these imports for TF2 transformations
import tf2_geometry_msgs  # Required for PointStamped transformations
import tf.transformations
from cv_bridge import CvBridge


# This class will be used to pick up objects from a table and place them in a virtual basket
class PickUpObject:
    def __init__(self, robot_name="locobot", init_node=False):
        # ---------- ROS NODE SETUP ----------
        self.robot_name = robot_name

        if init_node:
            rospy.init_node('pick_up_object', anonymous=False)

        # ---------- ROBOT SETUP ----------
        # Instead of using InterbotixLocobotXS, we'll create our own interfaces directly
        
        # Create the core interface for basic servo control
        self.core = InterbotixRobotXSCore(
            robot_model=f"{self.robot_name}_wx250s",
            robot_name=self.robot_name,
            init_node=False
        )
        
        # Create the arm interface
        self.arm = InterbotixArmXSInterface(
            core=self.core,
            robot_model="mobile_wx250s",
            group_name="arm"
        )
        
        # Create the gripper interface
        self.gripper = InterbotixGripperXSInterface(self.core, "gripper")
        
        # Create the perception interfaces
        self.pcl = InterbotixPointCloudInterface(
            filter_ns=f"{self.robot_name}/pc_filter",
            init_node=False
        )
        
        self.armtag = InterbotixArmTagInterface(
            armtag_ns=f"{self.robot_name}/armtag",
            apriltag_ns=f"{self.robot_name}/apriltag",
            init_node=False
        )

        # ---------- STATE VARIABLES ----------
        self.object_marker = None
        self.latest_keypoints = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()  # For converting between ROS images and OpenCV

        # Camera intrinsics placeholders
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------- ROS Interface SETUP ----------
        self._setup_marker_subscription()
        self._setup_keypoint_subscription()

        # ---------- SHUTDOWN HANDLER ----------
        rospy.on_shutdown(self.shutdown_handler)
    
    def _setup_marker_subscription(self):
        """Set up ROS subscription for object marker information"""
        self.object_marker_topic = rospy.get_param(
            '~object_marker_topic', 
            f'/{self.robot_name}/object_markers'
        )
        
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )

    def _setup_keypoint_subscription(self):
        """Set up subscription for keypoint data"""
        self.keypoint_topic = f'/{self.robot_name}/camera/yolo/keypoints'
        
        self.keypoint_sub = rospy.Subscriber(
            self.keypoint_topic,
            Float32MultiArray,
            self.keypoint_callback
        )

    def keypoint_callback(self, msg):
        """Process incoming keypoint data"""
        if not msg.data or len(msg.data) < 3:  # No valid data
            return
            
        # First value is the number of instances
        num_instances = int(msg.data[0])
        
        if num_instances > 0:
            # Store keypoints as 2D array: [[x1, y1], [x2, y2], ...]
            # Skip first value (number of instances) and reshape
            keypoints = []
            for i in range(1, len(msg.data), 2):
                if i+1 < len(msg.data):
                    keypoints.append([msg.data[i], msg.data[i+1]])
                    
            self.latest_keypoints = np.array(keypoints)
            if self.latest_keypoints.size > 0:
                rospy.loginfo(f"Received {len(self.latest_keypoints)} keypoints")
            
    def enable_keypoint_detection(self, enable=True):
        """Enable or disable keypoint detection"""
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
    
    def get_camera_info(self):
        """
        Get camera information from the RealSense camera_info topic
        
        Returns:
            tuple: (fx, fy) focal lengths or (None, None) if not available
        """
        try:
            camera_info_topic = f'/{self.robot_name}/camera/color/camera_info'
            camera_info = rospy.wait_for_message(
                camera_info_topic,
                sensor_msgs.msg.CameraInfo,
                timeout=2.0
            )
            
            # Extract focal lengths
            self.fx = camera_info.K[0]
            self.fy = camera_info.K[4]
            self.cy = camera_info.K[5]
            self.cx = camera_info.K[2]
            
            rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
            return self.fx, self.fy
            
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get camera info: {e}")
            return None, None

    def object_marker_callback(self, msg):
        """Process object marker messages and update history"""
        self.object_marker = msg

    def get_armtag(self):
        # position the arm such that the Apriltag is clearly visible to the camera
        self.arm.set_ee_pose_components(x=0.2, z=0.2, pitch=-math.pi/8.0)
        time.sleep(0.5)
        # get the transform of the AR tag
        self.armtag.find_ref_to_arm_base_transform(position_only=True)
        # move the arm out of the way of the camera
        self.arm.go_to_sleep_pose()

    def get_clusters(self):
        # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link'
        # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
        time.sleep(0.5)
        success, clusters = self.pcl.get_cluster_positions(
            ref_frame=f"{self.robot_name}/arm_base_link", 
            sort_axis="y", 
            reverse=True)
        
        return success, clusters
    
    def match_clusters_with_keypoints(self, clusters):
        """
        Match point cloud clusters with detected keypoints
        to determine grasp orientation for each cluster
        
        Args:
            clusters (list): List of detected object clusters
            
        Returns:
            list: Clusters with added orientation information
        """
        if not clusters or self.latest_keypoints is None or len(self.latest_keypoints) < 2:
            rospy.logwarn("Not enough data to match clusters with keypoints")
            return clusters
        
        # Create copy of clusters to add orientation data
        enhanced_clusters = []
        
        for cluster in clusters:
            # Get cluster 3D position
            x, y, z = cluster["position"]
            
            try:
                # Create a point in arm_base_link frame
                point = geometry_msgs.msg.PointStamped()
                point.header.frame_id = f"{self.robot_name}/arm_base_link"
                point.header.stamp = rospy.Time(0)
                point.point.x = x
                point.point.y = y
                point.point.z = z
                
                # Transform to camera frame to match with keypoints
                transformed_point = self.tf_buffer.transform(
                    point, 
                    f"{self.robot_name}/camera_color_optical_frame",
                    rospy.Duration(0.5)
                )
                
                # Get camera intrinsics if not available
                if not hasattr(self, 'fx') or self.fx is None:
                    self.get_camera_info()
                
                # Project 3D point to 2D image coordinates
                if transformed_point.point.z > 0:  # Point is in front of camera
                    pixel_x = int(self.cx + (transformed_point.point.x / transformed_point.point.z) * self.fx)
                    pixel_y = int(self.cy + (transformed_point.point.y / transformed_point.point.z) * self.fy)
                    
                    # Find closest keypoint pair to this projected point
                    best_distance = float('inf')
                    best_angle = 0
                    best_kp1 = None
                    best_kp2 = None
                    
                    # Loop through keypoint pairs
                    for i in range(0, len(self.latest_keypoints), 2):
                        if i+1 < len(self.latest_keypoints):
                            kp1 = self.latest_keypoints[i]
                            kp2 = self.latest_keypoints[i+1]
                            
                            # Calculate midpoint of keypoint pair
                            mid_x = (kp1[0] + kp2[0]) / 2
                            mid_y = (kp1[1] + kp2[1]) / 2
                            
                            # Calculate distance to projected point
                            distance = math.sqrt((pixel_x - mid_x)**2 + (pixel_y - mid_y)**2)
                            
                            if distance < best_distance:
                                best_distance = distance
                                best_kp1 = kp1
                                best_kp2 = kp2
                    
                    # If a good match was found (threshold can be adjusted)
                    if best_distance < 100 and best_kp1 is not None and best_kp2 is not None:  # Pixel distance threshold
                        # Calculate angle between keypoints for orientation
                        dx = best_kp2[0] - best_kp1[0]
                        dy = best_kp2[1] - best_kp1[1]
                        keypoint_angle = math.atan2(dy, dx)
                        
                        # Create direction vector from angle
                        dir_vector = geometry_msgs.msg.Vector3Stamped()
                        dir_vector.header.frame_id = f"{self.robot_name}/camera_color_optical_frame"
                        dir_vector.header.stamp = rospy.Time(0)
                        
                        # Create unit vector in keypoint direction
                        dir_vector.vector.x = math.cos(keypoint_angle)
                        dir_vector.vector.y = math.sin(keypoint_angle)
                        dir_vector.vector.z = 0.0
                        
                        # Transform direction to arm base frame
                        transformed_vector = self.tf_buffer.transform(
                            dir_vector, 
                            f"{self.robot_name}/arm_base_link",
                            rospy.Duration(0.5)
                        )
                        
                        # Calculate roll from transformed vector
                        vx = transformed_vector.vector.x
                        vy = transformed_vector.vector.y
                        roll = -math.atan2(vy, vx)
                        
                        # Normalize roll (-π/2 to π/2)
                        if roll > math.pi/2:
                            roll -= math.pi
                        elif roll < -math.pi/2:
                            roll += math.pi
                        
                        # Limit roll to what the arm can handle
                        roll = max(-math.pi/4, min(roll, math.pi/4))
                        
                        # Add orientation to cluster
                        enhanced_cluster = cluster.copy()
                        enhanced_cluster["orientation"] = {
                            "pitch": 0.5,  # Default grasp pitch
                            "roll": roll
                        }
                        enhanced_clusters.append(enhanced_cluster)
                        
                        rospy.loginfo(f"Matched cluster at ({x:.3f}, {y:.3f}, {z:.3f}) with keypoints, roll={roll:.2f}")
                        continue
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF error during cluster-keypoint matching: {e}")
            
            # If no match or error, just use the original cluster with default orientation
            enhanced_cluster = cluster.copy()
            enhanced_cluster["orientation"] = {
                "pitch": 0.5,
                "roll": 0.0
            }
            enhanced_clusters.append(enhanced_cluster)
        
        return enhanced_clusters

    def check_pick_up(self):
        """
        Check if object was picked up by verifying marker absence
        Added more robust detection mechanisms
        """
        # Check for missing marker (traditional approach)
        if self.object_marker is None:
            rospy.loginfo("Object marker is None - object picked up")
            return True
            
        # Additional checks for marker timeliness
        if hasattr(self.object_marker, 'header') and hasattr(self.object_marker.header, 'stamp'):
            # Check if marker message is old (more than 2 seconds)
            marker_age = (rospy.Time.now() - self.object_marker.header.stamp).to_sec()
            if marker_age > 2.0:
                rospy.loginfo(f"Object marker is stale (age: {marker_age:.1f}s) - considering object picked up")
                return True
        
        # The object is still detected - pickup failed
        rospy.loginfo(f"Object marker still present at position: "
                     f"[{self.object_marker.pose.position.x:.3f}, "
                     f"{self.object_marker.pose.position.y:.3f}, "
                     f"{self.object_marker.pose.position.z:.3f}]")
        return False
        
    def pick_up_object(self, clusters, max_attempts=3):
        """
        Pick up only the first detected cluster with retry logic
        
        Args:
            clusters (list): List of detected object clusters
            max_attempts (int): Maximum number of pickup attempts
            
        Returns:
            bool: True if object was picked up successfully, False otherwise
        """
        # move the robot back so it's centered and open the gripper
        self.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
        self.gripper.open()

        # Only pick up the first cluster
        if not clusters:
            rospy.logwarn("No clusters available to pick up")
            return False
         
        # Match clusters with keypoints to get orientation
        enhanced_clusters = self.match_clusters_with_keypoints(clusters)
            
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            rospy.loginfo(f"Pick up attempt {attempt}/{max_attempts}")
            
            cluster = enhanced_clusters[0]  # Get only the first cluster
            x, y, z = cluster["position"]
            
            # Get orientation if available, otherwise use defaults
            pitch = 0.5  # Default pitch (about 30 degrees)
            roll = 0.0    # Default roll
            
            if "orientation" in cluster:
                pitch = cluster["orientation"]["pitch"]
                roll = cluster["orientation"]["roll"]
            
            rospy.loginfo(f"Picking up object at position: x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch:.2f}, roll={roll:.2f}")
            
            # Move to position above object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=pitch, roll=roll)
            
            # Move down to object - slightly different positions on retries
            z_offset = 0 if attempt == 1 else (0.005 if attempt == 2 else -0.005)
            self.arm.set_ee_pose_components(x=x, y=y, z=z+z_offset, pitch=pitch, roll=roll)
            
            # Close gripper to grab object
            self.gripper.close(delay=1.5)
            
            # Lift object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=pitch, roll=roll)
            
            # Move to a position away from camera view 
            self.arm.set_ee_pose_components(x=0.3, y=-0.2, z=0.3, pitch=0.0)
            
            # Give a moment for the marker detection to update
            rospy.sleep(1.0)
            
            # Check if object was picked up
            # IMPORTANT: Do not set object_marker to None before this check
            if self.check_pick_up():
                rospy.loginfo(f"Successfully picked up object on attempt {attempt}")
                
                # Now we can explicitly set object_marker to None 
                # to ensure future operations know the object is gone
                self.object_marker = None
                
                # Move arm to final position
                self.arm.go_to_sleep_pose()
                return True
            else:
                rospy.logwarn(f"Failed to pick up object on attempt {attempt}")
                
                if attempt < max_attempts:
                    # Return to neutral position for next attempt
                    self.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.0)
                    self.gripper.open()
                    rospy.sleep(0.5)  # Short pause before next attempt
        
        # All attempts failed
        rospy.logerr(f"Failed to pick up object after {max_attempts} attempts")
        self.arm.go_to_sleep_pose()
        return False

    def calculate_pickup_target_from_keypoints(self):
        """
        Calculate pickup target position and orientation from keypoints
        
        Returns:
            tuple: (x, y, z, pitch, roll) coordinates and orientation for pickup in arm_base_link frame,
                   or None if unavailable
        """
        if self.latest_keypoints is None or len(self.latest_keypoints) < 2:
            rospy.logwarn("Not enough keypoints for pickup calculation")
            return None
        
        # Use the two available keypoints
        kp1 = self.latest_keypoints[0]
        kp2 = self.latest_keypoints[1]
        
        # Calculate midpoint between keypoints - this is where we'll pick
        mid_x = (kp1[0] + kp2[0]) / 2
        mid_y = (kp1[1] + kp2[1]) / 2
        
        # Calculate angle between keypoints for gripper orientation
        # This will help align the gripper with the object's orientation
        dx = kp2[0] - kp1[0]
        dy = kp2[1] - kp1[1]
        keypoint_angle = math.atan2(dy, dx)
        
        # Get depth from depth image or object marker
        target_depth = self.get_depth_at_pixel(mid_x, mid_y)
        if target_depth is None or target_depth <= 0:
            # break out if we don't have a valid depth value
            rospy.logwarn("No valid depth value for pickup target")
            return None
        else:
            rospy.loginfo(f"Using depth from image: {target_depth}m")
        
        # Safety checks on the depth value
        if target_depth < 0.1 or target_depth > 2.0:
            rospy.logwarn(f"Suspicious depth value: {target_depth}m, clamping to valid range")
            target_depth = max(0.1, min(target_depth, 2.0))
        
        # Make sure we have camera intrinsics
        if not hasattr(self, 'fx') or self.fx is None:
            self.get_camera_info()
            if not hasattr(self, 'fx') or self.fx is None:
                rospy.logerr("Failed to get camera intrinsics")
                return None
        
        try:
            # Create a Point message with the camera coordinates
            point = geometry_msgs.msg.PointStamped()
            point.header.frame_id = f"{self.robot_name}/camera_color_optical_frame"
            point.header.stamp = rospy.Time(0)
            
            # Calculate 3D position using pinhole camera model
            point.point.x = (mid_x - self.cx) * target_depth / self.fx
            point.point.y = (mid_y - self.cy) * target_depth / self.fy
            point.point.z = target_depth

            # Transform from camera frame to arm base frame
            transformed_point = self.tf_buffer.transform(
                point, 
                f"{self.robot_name}/arm_base_link",
                rospy.Duration(0.5)
            )
            
            # Extract coordinates
            x = transformed_point.point.x
            y = transformed_point.point.y
            z = transformed_point.point.z
            
            # Apply any necessary offsets for better grasping
            grasp_z_offset = 0.01  # Vertical adjustment (slightly above object)
            z += grasp_z_offset
            
            # Calculate pitch and roll for gripper orientation
            # Convert 2D keypoint angle to 3D gripper orientation
            # We need to transform the angle from camera frame to arm frame
            
            # First, create a unit vector in the direction of the keypoints in camera frame
            dir_vector = geometry_msgs.msg.Vector3Stamped()
            dir_vector.header.frame_id = f"{self.robot_name}/camera_color_optical_frame"
            dir_vector.header.stamp = rospy.Time(0)
            
            # Create a unit vector in the direction of the keypoints
            dir_vector.vector.x = math.cos(keypoint_angle)
            dir_vector.vector.y = math.sin(keypoint_angle)
            dir_vector.vector.z = 0.0
            
            # Transform direction vector to arm base frame
            try:
                transformed_vector = self.tf_buffer.transform(
                    dir_vector, 
                    f"{self.robot_name}/arm_base_link",
                    rospy.Duration(0.5)
                )
                
                # Extract vector components
                vx = transformed_vector.vector.x
                vy = transformed_vector.vector.y
                
                # Calculate desired gripper pitch (usually around 0.5 radians)
                pitch = 0.5  # Default pitch (about 30 degrees)
                
                # Calculate roll based on the vector orientation in arm frame
                roll = -math.atan2(vy, vx)
                
                # Normalize roll to be between -π/2 and π/2
                if roll > math.pi/2:
                    roll -= math.pi
                elif roll < -math.pi/2:
                    roll += math.pi
                    
                # Limit roll to what the arm can handle
                roll = max(-math.pi/4, min(roll, math.pi/4))
                
                rospy.loginfo(f"Calculated grasp orientation: pitch={pitch:.2f}, roll={roll:.2f}")
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Could not transform orientation: {e}")
                pitch = 0.5  # Default pitch
                roll = 0.0    # Default roll
            
            rospy.loginfo(f"Pickup target from keypoints: x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch:.2f}, roll={roll:.2f}")
            return (x, y, z, pitch, roll)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")
            return None

    def get_depth_at_pixel(self, x, y):
        """
        Get depth value at a specific pixel from the depth image
        
        Args:
            x (int): Pixel x coordinate
            y (int): Pixel y coordinate
            
        Returns:
            float: Depth in meters, or None if unavailable
        """
        try:
            # Get latest depth image
            depth_topic = f"/{self.robot_name}/camera/depth/image_rect_raw"
            depth_msg = rospy.wait_for_message(depth_topic, sensor_msgs.msg.Image, timeout=1.0)
            
            # Convert to OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # Convert to meters if needed (depends on your depth camera)
            # RealSense depth images are typically in millimeters
            depth_scale = 0.001  # For RealSense (1mm = 0.001m)
            
            # Get depth at pixel (clamp to image bounds)
            height, width = depth_image.shape
            x = max(0, min(int(x), width-1))
            y = max(0, min(int(y), height-1))
            
            # Get depth value (convert to meters)
            depth = float(depth_image[y, x]) * depth_scale
            
            # Check for invalid values
            if depth <= 0 or math.isnan(depth):
                # Try a small neighborhood
                kernel_size = 5
                x_start = max(0, x - kernel_size//2)
                x_end = min(width, x + kernel_size//2)
                y_start = max(0, y - kernel_size//2)
                y_end = min(height, y + kernel_size//2)
                
                neighborhood = depth_image[y_start:y_end, x_start:x_end]
                valid_depths = neighborhood[(neighborhood > 0)]
                
                if len(valid_depths) > 0:
                    depth = float(np.median(valid_depths)) * depth_scale
                else:
                    rospy.logwarn("No valid depth values found around pixel")
                    return None
                    
            return depth
                
        except Exception as e:
            rospy.logerr(f"Error getting depth at pixel: {e}")
            return None

    def run(self):
        """Pick up an object detected by the perception system"""
        # Get the arm tag to calibrate perception
        try:
            self.get_armtag()
        except Exception as e:
            rospy.logerr(f"Failed to get armtag: {str(e)}")
            return False
        
        # Ensure keypoint detection is enabled
        self.enable_keypoint_detection(True)
        
        # Make sure we have camera intrinsics
        self.get_camera_info()
        
        # Wait a moment for keypoints to be detected
        rospy.sleep(1.0)
            
        # Single attempt to pick up an object
        try:
            # Get clusters
            success, clusters = self.get_clusters()
            
            if not success or not clusters:
                rospy.logwarn("No object clusters detected")
                return False
            
            # Pick up the first object
            if self.pick_up_object(clusters):
                rospy.loginfo("Successfully picked up object")
                return True
            else:
                rospy.logwarn("Failed to pick up object")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error in object pickup: {str(e)}")
            return False
        
    # ---------- SHUTDOWN HANDLER ----------
    def shutdown_handler(self):
        """Handle shutdown - ensure the arm is in a safe position"""
        rospy.loginfo("PickUpObject shutting down, moving arm to safe position...")
        
        try:
            # Open gripper to drop any held objects
            self.gripper.open()
            rospy.sleep(0.5)  # Short pause to ensure gripper opens
            
            # Move to a safe intermediate position if the arm is extended
            try:
                # Check if arm is in a position that might be dangerous
                # This is a simple check - you may want more sophisticated detection
                current_positions = self.arm.get_joint_positions()
                if any(abs(pos) > 0.2 for pos in current_positions):
                    # Move to a safe intermediate position first
                    self.arm.set_ee_pose_components(x=0.2, z=0.2, pitch=0.0, moving_time=1.0)
                    rospy.sleep(0.5)
            except Exception:
                # If we can't get positions, still try to go to sleep
                pass
                
            # Move to sleep pose
            self.arm.go_to_sleep_pose()
            rospy.loginfo("Arm successfully moved to sleep pose")
            
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {str(e)}")


if __name__=="__main__":
    # Initialize the ROS node
    rospy.init_node('pick_up_object', anonymous=False)
    
    # Create and run the PickUpObject class
    po = PickUpObject(init_node=False)
    
    # Run the demo
    success = po.run()