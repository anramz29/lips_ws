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
        # check if the object marker is present
        if self.object_marker is None:
            rospy.logwarn("No object marker detected, robot picked up objects")
            return True
        else:
            rospy.loginfo("Object marker detected, robot did not pick up objects")
            return False
        
    # ---------- ACTION METHODS ----------
    
    def pick_up_object(self, clusters):
        """
        Pick up only the first detected cluster
        
        Args:
            clusters (list): List of detected object clusters
        """
        # move the robot back so it's centered and open the gripper
        self.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
        self.gripper.open()

        # Only pick up the first cluster
        if clusters:
            cluster = clusters[0]  # Get only the first cluster
            x, y, z = cluster["position"]
            
            rospy.loginfo(f"Picking up object at position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Move to position above object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            
            # Move down to object
            self.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
            
            # Close gripper to grab object
            self.gripper.close(delay=1.5)
            
            # Lift object
            self.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            
            # Move to sleep pose
            self.arm.go_to_sleep_pose()

            self.object_marker = None
            
            return True
        else:
            rospy.logwarn("No clusters available to pick up")
            return False

    # ---------- MAIN LOOP ----------
        
    def run(self):
        # get the arm out of the way of the camera
        self.get_armtag()
        while not rospy.is_shutdown():
            # get the clusters of objects
            success, clusters = self.get_clusters()
            if not success:
                rospy.logwarn("No clusters detected")
                continue
            # pick up the objects
            self.pick_up_object(clusters)
            rospy.sleep(2)
            # check if the object marker is present
            if self.check_pick_up():
                break
            else:
                continue

        rospy.loginfo("Object pickup complete")
        rospy.spin()
        

        return True


if __name__=="__main__":
    po = PickUpObject()
    po.run()


