import math
import time
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from interbotix_xs_modules.locobot import InterbotixLocobotXS
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
        
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")

        # ---------- STATE VARIABLES ----------

        self.object_marker = None
        self.last_object_marker = None
        self.angle = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

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

        if self.object_marker is not None:
            self.last_object_marker = self.object_marker

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
        success, clusters = self.bot.pcl.get_cluster_positions(
            ref_frame=f"{self.robot_name}/arm_base_link", 
            sort_axis="y", 
            reverse=True
        )
        
        # log number of clusters found
        if success:
            rospy.loginfo(f"Found {len(clusters)} clusters")
            x,y,z = clusters[0]['position']
            rospy.loginfo(f"Cluster position: x={x}, y={y}, z={z}")
        else:
            rospy.logerr("Failed to get cluster positions")
            return []
            
        return success, x,y,z

    def shutdown_handler(self):
        """Shutdown handler to ensure a clean exit"""
        rospy.loginfo("Shutting down...")
        self.enable_keypoint_detection(False)
        self.bot.arm.go_to_sleep_pose()
        rospy.sleep(1.0)  # Short pause for stability
        rospy.loginfo("Shutdown complete")

    def pick_object(self, approach_height=0.15):
        """Main function to pick up an object using the robot arm."""
        
        # Enable keypoint detection
        if not self.enable_keypoint_detection(True):
            rospy.logerr("Failed to enable keypoint detection")
            return False
        
        # Wait for the object marker to be available
        while self.object_marker is None:
            rospy.loginfo("Waiting for object marker...")
            rospy.sleep(0.5)

        # Get the position of the object marker
        x, y, _ = self.get_clusters()

        # turn the angle into radians
        if self.angle is None:
            rospy.logerr("No angle detected for object")
            return False
        angle_radians = math.radians(self.angle)
        rospy.loginfo_once(f"Using angle: {self.angle} radians")

        self.bot.gripper.open()
        self.bot.go_to_home_pose()


        # Move the arm to the approach height above the object
        rospy.loginfo_once(f"Moving arm to x={x}, y={y}, z={approach_height} with angle {angle_radians}")
        move_1_success = self.bot.arm.set_ee_pose_components(
            x=x,
            y=y,
            z=approach_height, 
            pitch=1.5, 
            yaw=angle_radians
        )

        if not move_1_success:
            rospy.logerr("Failed to move arm to approach height")
            self.bot.arm.go_to_sleep_pose()
            return False
        
        # Wait for the arm to reach the position
        move_2_success = self.bot.arm.set_ee_pose_components(
            x=x, 
            y=y, 
            z=0.05, 
            pitch=1.5, 
            yaw=angle_radians
        )

        if not move_2_success:
            rospy.logerr("Failed to move arm to object height")
            self.bot.arm.go_to_sleep_pose()
            return False

        # Wait for the arm to reach the position
        rospy.sleep(1.0)

        # Close the gripper to pick up the object
        self.bot.gripper.close()

        # Wait for the gripper to close
        rospy.sleep(1.0)

        # Move the arm back to sleep pose
        self.bot.arm.go_to_sleep_pose()

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
    po.bot.arm.go_to_sleep_pose()
    
if __name__ == "__main__":
    main()