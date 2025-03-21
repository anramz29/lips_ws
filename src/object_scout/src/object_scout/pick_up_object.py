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
import collections


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
        self.latest_angle = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.angle_history = collections.deque(maxlen=10)
        self.last_valid_angle = None

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
        try:
            if not msg.data or len(msg.data) < 3:  # No valid data
                return
                
            # First value is the number of instances
            num_instances = int(msg.data[0])
            
            if num_instances > 0:
                # Store the angle and keypoints
                # The angle is the second value in the message
                self.angle = float(msg.data[1])  # Ensure it's a float
                
                # Extract keypoints starting from the third value
                keypoints = []
                for i in range(2, len(msg.data), 2):
                    if i+1 < len(msg.data):
                        keypoints.append([msg.data[i], msg.data[i+1]])
                        
                # Store the first set of angle
                self.latest_angle = self.angle

                
        except Exception as e:
            rospy.logerr(f"Error processing keypoint data: {e}")

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


    def shutdown_handler(self):
        """Shutdown handler to ensure a clean exit"""
        rospy.loginfo("Shutting down...")
        self.enable_keypoint_detection(False)
        self.arm.go_to_sleep_pose()
        rospy.sleep(1.0)  # Short pause for stability
        rospy.loginfo("Shutdown complete")

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