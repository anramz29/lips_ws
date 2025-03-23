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
        self.latest_angle = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.last_valid_angle = None

        # ---------- ROS Interface SETUP ----------

        # Set up the marker subscription for object detection
        self._setup_ros_communication()
    
        # ---------- SHUTDOWN HANDLER ----------
        rospy.on_shutdown(self.shutdown_handler)
    
    def _setup_ros_communication(self):
        """Set up ROS subscription for object marker information"""
        self.object_marker_topic = rospy.get_param(
            f'/{self.robot_name}/object_markers'
        )
        
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )
        
        self.keypoint_sub = rospy.Subscriber(
            f'/{self.robot_name}/camera/yolo/keypoints',
            Float32MultiArray,
            self.keypoint_callback
        )

    # ---------- Enable Key Points----------

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

    def object_marker_callback(self, msg):
        """Process object marker messages and update history"""
        self.object_marker = msg
    
        

    def keypoint_callback(self, msg):
        """Process keypoint messages and extract the angle from the first detection.
        
        Args:
            msg (Float32MultiArray): Message containing keypoint data and angles
        """
        try:
            # Check if the message contains at least one detection
            if len(msg.data) > 0 and msg.data[0] >= 1:
                num_detections = int(msg.data[0])
                
                if num_detections >= 1:
                    # Get data for the first detection
                    detection_data = msg.data[1:]
                    
                    self.angle = detection_data[11]

        except IndexError as e:
            rospy.logerr(f"Keypoint data is malformed: {e}")
            return
        

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
    
    def calculate_yaw(self):
        """Calculate the perpendicular yaw angle keypoint angle 

        args:
            None

        returns:
            yaw (float): The calculated yaw angle in radians
        
        """

        



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