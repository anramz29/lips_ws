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
        self.angle = None
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
        """Process incoming keypoint data
        
        Message format: [num_instances, angle, x1, y1, x2, y2, ...]
        """
        if not msg.data or len(msg.data) < 3:  # No valid data
            return
            
        # First value is the number of instances
        num_instances = int(msg.data[0])
        
        if num_instances > 0:
            # Store the angle and keypoints
            # The angle is the second value in the message
            self.angle = msg.data[1]
            
            # Extract keypoints starting from the third value
            keypoints = []
            for i in range(2, len(msg.data), 2):
                if i+1 < len(msg.data):
                    keypoints.append([msg.data[i], msg.data[i+1]])
                    
            # Store the angle and keypoints
            self.latest_angle = self.angle      
            self.latest_keypoints = np.array(keypoints)
            
            # Log the data once

            
                
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
        
        # log number of clusters found
        if success:
            rospy.loginfo(f"Found {len(clusters)} clusters")
        
        return success, clusters
        

    def normalize_angle(self):
        """
        Normalize an angle to the -π to π range
        """
        
        # Normalize angle (in radians) to -π to π range
        normalized_angle = math.atan2(math.sin(self.angle), math.cos(self.angle))
        
        # If the angle is greater than π/2 or less than -π/2, we can flip it by ±π
        # This gives us the equivalent angle with smallest magnitude
        if normalized_angle > math.pi/2:
            optimal_yaw = normalized_angle - math.pi
        elif normalized_angle < -math.pi/2:
            optimal_yaw = normalized_angle + math.pi
        else:
            optimal_yaw = normalized_angle
            
        rospy.loginfo(f"Optimized yaw: {optimal_yaw:.4f} rad ({math.degrees(optimal_yaw):.2f}°) from original {normalized_angle:.4f} rad")
        
        return optimal_yaw

    def pick_object(self, hover_height=0.15, approach_height=0.01, go_to_sleep_after=True):
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

        # go to sleep pose if requested
        if go_to_sleep_after:
            self.arm.go_to_sleep_pose()

        self.gripper.open()

        rospy.sleep(1.0)  # Short pause for stability
        
        # 3. Get clusters to identify objects
        success, clusters = self.get_clusters()
        if not success or len(clusters) == 0:
            rospy.logerr("No objects detected")
            return False
        
        self.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
        self.gripper.open()
            
        # 4. Get the first cluster (assume it's our target)
        target_cluster = clusters[0]
        rospy.loginfo(f"Target object at position: {target_cluster}")
        x, y, z = target_cluster["position"]

  
        # 5. Move to hover position above object
        rospy.loginfo("Moving to hover position...")
        hover_success = self.arm.set_ee_pose_components(
            x=x, y=y, z=0.25, moving_time=1.5,
        )

        rospy.sleep(0.5)  # Short pause for stability

        
        approach_success = self.arm.set_ee_cartesian_trajectory(pitch=math.pi/2)
        
        if not hover_success:
            rospy.logerr("Failed to reach hover position")
            self.arm.go_to_sleep_pose()
            return False
        
        # 6. Get optimal yaw angle for grasping
        optimal_yaw = self.normalize_angle()
            
        approach_success = self.arm.set_ee_cartesian_trajectory(yaw=optimal_yaw)

        if not approach_success:
            rospy.logerr("Failed to approach object")
            self.arm.go_to_sleep_pose()
            return False
        
        # 9. Lower to approach height with Cartesian trajectory for straight-line motion
        rospy.loginfo("Approaching object...")
        approach_success = self.arm.set_ee_cartesian_trajectory(z=approach_height)
        
        if not approach_success:
            rospy.logerr("Failed to approach object")
            self.arm.go_to_sleep_pose()
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
            self.arm.go_to_sleep_pose()
            return False
        
        # 12. Go to sleep pose if requested
        if go_to_sleep_after:
            rospy.loginfo("Moving to sleep pose...")
            self.arm.go_to_sleep_pose()
            
        rospy.loginfo("Successfully picked up the object!")
        return True

    def shutdown_handler(self):
        """Handle shutdown cleanup"""
        rospy.loginfo("PickUpObject shutting down, moving arm to safe position...")
        try:
            # Open gripper first (safer in case it's holding something)
            self.gripper.open()
            rospy.sleep(0.5)  # Short pause to ensure gripper opens
            
            # Move arm to sleep pose
            self.arm.go_to_sleep_pose()
            rospy.loginfo("Arm successfully moved to sleep pose")
        except Exception as e:
            rospy.logerr(f"Error during shutdown cleanup: {e}")


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