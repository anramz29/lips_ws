import math
import time
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.arm import InterbotixArmXSInterface
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped


class PickUpObject:
    def __init__(self, robot_name="locobot", init_node=False):
        # ---------- ROS NODE SETUP ----------
        self.robot_name = robot_name

        if init_node:
            rospy.init_node('pick_up_object', anonymous=False)

        # ---------- ROBOT SETUP ----------
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
        self.bridge = CvBridge()

        # Camera intrinsics (initialized in get_camera_info)
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
            
            # Extract focal lengths and principal point
            self.fx = camera_info.K[0]
            self.fy = camera_info.K[4]
            self.cx = camera_info.K[2]
            self.cy = camera_info.K[5]
            
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

    def calculate_orientation_from_keypoints(self, keypoints):
        """
        Calculate orientation angle (in radians) from two keypoints
        
        Args:
            keypoints: Array of 2D keypoints [x, y]
            
        Returns:
            float: Angle in radians, or None if not enough keypoints
        """
        if keypoints is None or len(keypoints) < 2:
            rospy.logwarn("Not enough keypoints to calculate orientation")
            return None
            
        # Extract the first two keypoints
        p1 = keypoints[0]
        p2 = keypoints[1]
        
        # Calculate vector direction
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        # Calculate angle with respect to horizontal
        angle = math.atan2(dy, dx)
        
        rospy.loginfo(f"Keypoint orientation: {angle} radians ({math.degrees(angle)} degrees)")
        return angle

    def project_pixel_to_3d(self, pixel_x, pixel_y, depth):
        """
        Project a 2D pixel point to 3D using camera intrinsics and depth
        
        Args:
            pixel_x: x-coordinate in the image
            pixel_y: y-coordinate in the image
            depth: depth value in meters
            
        Returns:
            tuple: (x, y, z) 3D point in camera frame
        """
        # Make sure camera intrinsics are available
        if None in (self.fx, self.fy, self.cx, self.cy):
            success = self.get_camera_info()
            if not success:
                rospy.logerr("Failed to get camera intrinsics for projection")
                return None
                
        # Project from image coordinates to 3D
        x = (pixel_x - self.cx) * depth / self.fx
        y = (pixel_y - self.cy) * depth / self.fy
        z = depth
        
        return (x, y, z)

    def transform_point(self, point, from_frame, to_frame):
        """
        Transform a 3D point from one frame to another
        
        Args:
            point: (x, y, z) tuple
            from_frame: source frame ID
            to_frame: target frame ID
            
        Returns:
            tuple: transformed (x, y, z)
        """
        # Create a PointStamped message
        point_stamped = PointStamped()
        point_stamped.header.frame_id = from_frame
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        
        try:
            # Transform the point
            transformed = self.tf_buffer.transform(point_stamped, to_frame, rospy.Duration(1.0))
            return (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")
            return None

    def keypoints_to_3d_pose(self, keypoints, cluster_center):
        """
        Convert 2D keypoints and known cluster to a 3D pose for the gripper
        
        Args:
            keypoints: Array of 2D keypoints [[x1,y1], [x2,y2]]
            cluster_center: (x, y, z) position of the object
            
        Returns:
            tuple: (x, y, z, roll, pitch, yaw) for gripper pose
        """
        if keypoints is None or len(keypoints) < 2 or cluster_center is None:
            rospy.logwarn("Invalid keypoints or cluster data")
            return None
        
        # Calculate orientation from keypoints
        orientation = self.calculate_orientation_from_keypoints(keypoints)
        if orientation is None:
            return None
            
        # The cluster center gives us the x, y, z position
        x, y, z = cluster_center
        
        # For top-down grasping, we want:
        # - Pitch set to pi/2 (vertical approach)
        # - Roll adjusted based on the keypoint orientation
        # - Yaw is irrelevant for a vertical grasp
        
        # Adjust roll based on the 2D orientation
        roll = 0.0  # This depends on your robot's coordinate system
        pitch = math.pi/2   # Vertical approach
        yaw = orientation        # Not used for vertical grasps
        
        return (x, y, z, roll, pitch, yaw)

    def pick_object(self, hover_height=0.1, approach_height=0.05, go_to_sleep_after=True):
        """
        Complete pick-up sequence for objects using keypoint orientation
        
        Args:
            hover_height: Height above object for pre-grasp position
            approach_height: Final height for grasping
            
        Returns:
            bool: Success flag
        """
        # 1. Enable keypoint detection if not already enabled
        self.enable_keypoint_detection(True)
        
        # 2. Calibrate the system using the armtag
        self.get_armtag()
        
        # 3. Get clusters to identify objects
        success, clusters = self.get_clusters()
        if not success or len(clusters) == 0:
            rospy.logerr("No objects detected")
            return False
            
        # 4. Get the first cluster (assume it's our target)
        target_cluster = clusters[0]
        rospy.loginfo(f"Target object at position: {target_cluster}")
        
        # 5. Make sure we have keypoints
        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while (self.latest_keypoints is None or len(self.latest_keypoints) < 2) and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for keypoints...")
            rospy.sleep(0.5)
            
        if self.latest_keypoints is None or len(self.latest_keypoints) < 2:
            rospy.logerr("Failed to get valid keypoints")
            return False
            
        # 6. Calculate gripper pose from keypoints and cluster
        gripper_pose = self.keypoints_to_3d_pose(self.latest_keypoints, target_cluster)
        if gripper_pose is None:
            rospy.logerr("Failed to calculate gripper pose")
            return False
            
        x, y, z, roll, pitch, yaw = gripper_pose
        
        # 7. Open the gripper
        self.gripper.open()
        
        # 8. Move to hover position above object
        rospy.loginfo("Moving to hover position...")
        hover_success = self.arm.set_ee_pose_components(
            x=x, y=y, z=z+hover_height,
            roll=roll, pitch=pitch, yaw=yaw
        )
        
        if not hover_success:
            rospy.logerr("Failed to reach hover position")
            return False
            
        # 9. Lower to approach height with Cartesian trajectory for straight-line motion
        rospy.loginfo("Approaching object...")
        approach_success = self.arm.set_ee_cartesian_trajectory(z=-hover_height+approach_height)
        
        if not approach_success:
            rospy.logerr("Failed to approach object")
            return False
            
        # 10. Close the gripper to grasp the object
        rospy.loginfo("Grasping object...")
        self.gripper.close()
        rospy.sleep(0.5)  # Wait for the gripper to close
        
        # 11. Lift the object
        rospy.loginfo("Lifting object...")
        lift_success = self.arm.set_ee_cartesian_trajectory(z=hover_height)
        
        if not lift_success:
            rospy.logerr("Failed to lift object")
            return False
        
        # 12. Go to sleep pose if requested
        if go_to_sleep_after:
            rospy.loginfo("Moving to sleep pose...")
            self.arm.go_to_sleep_pose()
            
        rospy.loginfo("Successfully picked up the object!")
        return True

    def shutdown_handler(self):
        """Handle shutdown cleanup"""
        rospy.loginfo("Shutting down PickUpObject")
        # Open gripper on shutdown
        try:
            self.gripper.open()
        except:
            pass

def main():
    # Initialize the ROS node
    rospy.init_node('pick_up_object', anonymous=False)
    
    # Create the PickUpObject instance
    po = PickUpObject(init_node=False)
    
    # Run the pickup sequence
    success = po.pick_object()
    
    if success:
        rospy.loginfo("Object pickup completed successfully")
    else:
        rospy.logerr("Object pickup failed")
    
    # Put the arm in a safe position
    po.arm.go_to_sleep_pose()
    
if __name__ == "__main__":
    main()