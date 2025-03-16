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

        # ---------- ROS Interface SETUP ----------
        self._setup_marker_subscription()

        # ---------- SHUTDOWN HANDLER ----------
        rospy.on_shutdown(self.shutdown_handler)
    
    # The rest of your methods remain unchanged, but you'll access components directly
    # instead of through self.bot (e.g., self.arm instead of self.bot.arm)
    
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
        # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link';
        # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
        time.sleep(0.5)
        success, clusters = self.pcl.get_cluster_positions(
            ref_frame="locobot/arm_base_link", 
            sort_axis="y", 
            reverse=True)
        
        return success, clusters
    

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
        
    # ---------- ACTION METHODS ----------
    
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
            
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            rospy.loginfo(f"Pick up attempt {attempt}/{max_attempts}")
            
            cluster = clusters[0]  # Get only the first cluster
            x, y, z = cluster["position"]
            
            rospy.loginfo(f"Picking up object at position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Move to position above object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            
            # Move down to object - slightly different positions on retries
            z_offset = 0 if attempt == 1 else (0.005 if attempt == 2 else -0.005)
            self.arm.set_ee_pose_components(x=x, y=y, z=z+z_offset, pitch=0.5)
            
            # Close gripper to grab object
            self.gripper.close(delay=1.5)
            
            # Lift object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            
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

    # ---------- MAIN LOOP ----------
        
    def run(self):
        """Pick up an object detected by the perception system"""
        # Get the arm tag to calibrate perception
        try:
            self.get_armtag()
        except Exception as e:
            rospy.logerr(f"Failed to get armtag: {str(e)}")
            return False
            
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
    po = PickUpObject()
    po.run()


