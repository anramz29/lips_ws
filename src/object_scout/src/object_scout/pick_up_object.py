import math
import rospy
from visualization_msgs.msg import Marker
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Float32MultiArray, Float32
import tf2_ros
from cv_bridge import CvBridge
import numpy as np

class PickUpObject:
    def __init__(self, robot_name, fine_approacher, 
                keypoint_angle_topic, enable_keypoint_detection_service,
                init_node=False):
        
        # ---------- ROS NODE SETUP ----------
        self.robot_name = robot_name
        self.keypoint_angle_topic = keypoint_angle_topic
        self.enable_keypoint_detection_service = enable_keypoint_detection_service

        if init_node:
            rospy.init_node('pick_up_object', anonymous=False)

        # ---------- ROBOT SETUP ----------
        
        self.bot = InterbotixLocobotXS(
            "locobot_wx250s", 
            arm_model="mobile_wx250s",
            init_node=False
        )

        # ---------- STATE VARIABLES ----------

        self.fine_approacher = fine_approacher
        self.keypoint_angle = None
        self.gripper_joint_name = "gripper"
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
        
        self.keypoint_angle_sub = rospy.Subscriber(
            self.keypoint_angle_topic,
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
        
        try:
            # Wait for service to be available
            rospy.wait_for_service(self.enable_keypoint_detection_service, timeout=3.0)
            
            # Call the service
            set_enabled = rospy.ServiceProxy(self.enable_keypoint_detection_service, SetBool)
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


    def angle_callback(self, msg):
        """Process angle messages and normalize the angle to a workable range.
        
        Args:
            msg (Float32): The message containing the perpendicular angle of the object.
        """
        try:
            raw_angle = msg.data
            
            # Normalize angle to stay within workable limits
            # If angle is between 85-90 or -85-(-90), cap it at 84 degrees
            if abs(raw_angle) > 76.0:
                # Keep the sign but cap the magnitude
                self.angle = 75.0 * (1 if raw_angle > 0 else -1)
                rospy.loginfo_once(f"Normalized angle from {raw_angle} to {self.angle}")
            else:
                self.angle = raw_angle
                
            rospy.loginfo_once(f"Angle: {self.angle}, in radians: {math.radians(self.angle)}")

        except Exception as e:
            rospy.logerr(f"Error processing angle: {e}")
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


    def check_gripper_has_object(self):
        """
        Check if the gripper has successfully grasped an object.
        
        When a gripper closes on an object, it should stop at a position between
        fully open and fully closed. If the gripper reaches its minimum position (fully closed),
        it likely missed the object.
        
        Returns:
            bool: True if an object is detected in the gripper, False otherwise
        """
        try:
            # Get the current state of the gripper joint
            gripper_state = self.bot.dxl.robot_get_single_joint_state(self.gripper_joint_name)
            position = gripper_state["position"]
            
            # Define thresholds - these will need adjustment based on testing
            fully_closed_threshold = 0.02  # Just above 0.0
            
            # The normalized position value might be in a different range than raw values
            # Log the actual position for debugging
            rospy.loginfo(f"Gripper position after closing: {position}")
            
            # If gripper is almost fully closed, it likely missed the object
            # If it's partially closed, it likely has an object
            if position <= fully_closed_threshold:
                rospy.logwarn("Gripper appears fully closed - no object detected")
                return False
            else:
                rospy.loginfo(f"Object detected in gripper - position: {position}")
                return True
                
        except Exception as e:
            rospy.logerr(f"Error checking gripper state: {e}")
            return False

    def find_working_pitch_for_both(self, x, y, grab_height, angle_radians):
        """Find a pitch angle that works for both approach and grasp heights."""
        
        pitches = [1.5, 1.3, 1.1, 0.9, 0.7]
        working_pitch = None
        
        for pitch in pitches:
            rospy.loginfo(f"Testing pitch {pitch} for final ee position")
            
            # Test approach position with this pitch
            try:
                approach_result = self.bot.arm.set_ee_pose_components(
                    x=x, y=y, z=grab_height, pitch=pitch, yaw=angle_radians, execute=False
                )
                
                # If approach position works, test grab position with same pitch
                if approach_result[1]:
                    working_pitch = pitch
                    rospy.loginfo(f"Pitch {pitch} works for approach")
                    return True, working_pitch
                else:
                    rospy.loginfo(f"Pitch {pitch} doesn't work for approach")
                    
            except Exception as e:
                rospy.logwarn(f"Error testing pitch {pitch}: {e}")
        
        # If we get here, no pitch worked for both positions
        rospy.logerr("No pitch found that works for both positions")
        return False, None

    def pick_object(self, approach_height=0.15, grab_height=0.02):
        """Main function to pick up an object using the robot arm."""
        rospy.sleep(2.0) # Wait for keypoint detection to stabilize

        # Get the position of the object marker
        success, x, y, z = self.get_clusters()

        if not success:
            rospy.logerr("Failed to get cluster positions")
            return False

        # turn the angle into radians
        if self.angle is None:
            rospy.logerr("No angle detected for object")
            return False
        angle_radians = math.radians(self.angle)
        rospy.loginfo_once(f"Using angle: {angle_radians} radians")

        # Prepare the arm
        self.bot.gripper.open()
        self.bot.arm.go_to_home_pose()
        
        # Calculate grab height with safety margin
        safe_grab_height = max(z + grab_height, -0.08)  # Never go below -0.08m
        
        # Find a pitch that works for both positions
        success, working_pitch = self.find_working_pitch_for_both(
            x, y, safe_grab_height, angle_radians
        )

        if not success:
            rospy.logerr("Failed to find a pitch that works for both positions")
            return False
        
        # Now execute the movements with the consistent pitch
        
        # First, move to approach position
        rospy.loginfo(f"Moving arm to x={x}, y={y}, z={approach_height} with angle {angle_radians}, pitch={working_pitch}")
        
        move_1_success = self.bot.arm.set_ee_pose_components(
            x=x,
            y=y,
            z=approach_height, 
            pitch=working_pitch, 
            yaw=angle_radians
        )

        if not move_1_success:
            rospy.logerr("Failed to move arm to approach height")
            self.bot.arm.go_to_sleep_pose()
            return False
        
        # Wait for the arm to stabilize
        rospy.sleep(1.0)
        
        # Then move to grab position with the SAME pitch
        rospy.loginfo(f"Moving arm to z={safe_grab_height} with angle {angle_radians}, maintaining pitch={working_pitch}")
        
        move_2_success = self.bot.arm.set_ee_cartesian_trajectory(
            z=-(approach_height - safe_grab_height))
        
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
        
        # Check if an object was successfully grasped
        object_grasped = self.check_gripper_has_object()
        
        if not object_grasped:
            rospy.logerr("Failed to grasp object - gripper appears to be fully closed")
            self.bot.arm.go_to_home_pose()
            self.bot.arm.go_to_sleep_pose()
            return False
        
        # Object successfully grasped
        rospy.loginfo("Object successfully grasped!")
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()

        return True
    
    def pick_object_with_retries(self, max_retries=3, retry_delay=2.0):
        """
        Attempt to pick up an object with multiple retries if gripper remains closed after the pick attempt.
        
        
        Args:
            max_retries (int): Maximum number of attempts to pick up the object
            retry_delay (float): Delay between retry attempts in seconds
            
        Returns:
            bool: True if object was picked up successfully, False otherwise
        """

        # Enable keypoint detection
        if not self.enable_keypoint_detection(True):
            rospy.logerr("Failed to enable keypoint detection")
            return False
        
        rospy.sleep(3.0)


        for attempt in range(max_retries):
            # Try to pick up the object (now includes gripper check)
            success = self.pick_object()
            
            if success:
                rospy.loginfo(f"Successfully picked up the object on attempt {attempt+1}")
                self.enable_keypoint_detection(False)
                return True
            else:
                rospy.logwarn(f"Failed to pick up object on attempt {attempt+1}. Retrying...")
                rospy.sleep(retry_delay)

        rospy.logerr(f"Failed to pick up object after {max_retries} attempts")
        self.enable_keypoint_detection(False)
        return False
    

    
def main():
    # Initialize the ROS node
    rospy.init_node('pick_up_object', anonymous=False)
    
    # Create the PickUpObject instance
    po = PickUpObject(init_node=False)
    
    # Run the pickup sequence
    success = po.pick_object_with_retries()
    
    if success:
        rospy.loginfo("Object pickup completed successfully")
    else:
        rospy.logerr("Object pickup failed")
    
    # Put the arm in a safe position
    po.bot.arm.go_to_sleep_pose()
    
if __name__ == "__main__":
    main()