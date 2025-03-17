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
    

if __name__=="__main__":
    # Initialize the ROS node
    rospy.init_node('pick_up_object', anonymous=False)
    
    # Create and run the PickUpObject class
    po = PickUpObject(init_node=False)
    
    # Run the demo
    success = po.run()