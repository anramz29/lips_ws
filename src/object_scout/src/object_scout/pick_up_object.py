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
from std_msgs.msg import Float32MultiArray, Float32
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
            "~object_marker_topic",
            f'/{self.robot_name}/object_markers'
        )
        
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )
        
        self.keypoint_sub = rospy.Subscriber(
            f'/{self.robot_name}/camera/yolo/object_angle',
            Float32,
            self.angle_callback
        )

    

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
    
        

    def angle_callback(self, msg):
        """Process angle messages and extract the angle from the first detection.
        
        Args:
            msg (Float32): The message containing the perpendicular angle of the object.
        """
        try:
            # make sure it's the first message
            if self.angle is None:
                self.angle = msg.data

            rospy.loginfo_once(f"Angle: {self.angle}, in radians: {math.radians(self.angle)}")

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

    def shutdown_handler(self):
        """Shutdown handler to ensure a clean exit"""
        rospy.loginfo("Shutting down...")
        self.enable_keypoint_detection(False)
        self.arm.go_to_sleep_pose()
        rospy.sleep(1.0)  # Short pause for stability
        rospy.loginfo("Shutdown complete")

    def pick_object(self, approach_height=0.15, gripper_offset=0.05, pitch=1.5, max_attempts=3):
        """Main function to pick up an object using the robot arm."""
        
        # Enable keypoint detection
        if not self.enable_keypoint_detection(True):
            rospy.logerr("Failed to enable keypoint detection")
            return False
        
        # Wait for the object marker to be available
        while self.object_marker is None:
            rospy.loginfo("Waiting for object marker...")
            rospy.sleep(0.5)


        
        # Get clusters from the point cloud
        success, clusters = self.get_clusters()

        if not success or len(clusters) == 0:
            rospy.logerr("No clusters found")
            return False
        
        # Extract the first cluster position
        cluster_position = clusters[0]

        # convert self.angle to radians and check if it is valid
        if self.angle is not None:
            self.angle = math.radians(self.angle)
            if self.angle < -math.pi or self.angle > math.pi:
                rospy.logerr("Invalid angle detected")
                return False
            
        self.arm.go_to_home_pose()
        self.gripper.open()

        x, y, z = cluster_position['position']


        success = self.arm.set_ee_pose_components(
            x=x,
            y=y,
            z=approach_height,
        )


        if not success:
            rospy.logerr("Failed to move arm to object position")
            self.arm.go_to_sleep_pose()
            return False
        
        success = self.arm.set_ee_cartesian_trajectory(
            pitch = -1.5,
        )

        success = self.arm.set_ee_pose_components(
            yaw = self.angle
        )

        # Move the arm to the approach position
        success = self.arm.set_ee_cartesian_trajectory(
            y=-(approach_height - gripper_offset),  # Move down to the gripper offset
        )

        if not success:
            rospy.logerr("Failed to move arm to gripper offset position")
            self.arm.go_to_sleep_pose()
            return False

        self.gripper.close()

        self.arm.go_to_sleep_pose()  

        # check if the object marker is still present
        if self.object_marker is None:
            rospy.logerr("Object marker not found after pickup")
            return False

        return True
    
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