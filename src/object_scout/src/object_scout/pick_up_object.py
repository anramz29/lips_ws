import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import rospy
from visualization_msgs.msg import Marker


class PickUpObject:
    def __init__(self, init_node=False):
        if init_node:
            rospy.init_node('pick_up_object', anonymous=False)

        # ---------- ROBOT SETUP ----------
        
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")

        # ---------- STATE VARIABLES ----------
        self.object_marker = None

        # ---------- ROS Interface SETUP ----------

        self._setup_marker_subscription()

    # ---------- ROS COMMUNICATION SETUP ----------

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

    # ---------- CALLBACK METHODS ----------

    def object_marker_callback(self, msg):
        """
        Process object marker messages and update history
        
        Args:
            msg (Marker): Marker message from object detection
        """
        # Store the latest marker
        self.object_marker = msg


    # ---------- PERCEPTION METHODS ----------

    def get_armtag(self):
        # position the arm such that the Apriltag is clearly visible to the camera
        self.bot.arm.set_ee_pose_components(x=0.2, z=0.2, pitch=-math.pi/8.0)
        # sleep half a second to give time for the arm to settle after moving
        time.sleep(0.5)
        # get the transform of the AR tag relative to the camera frame; based on that transform,
        # publish a new transform from the 'locobot/plate_link' to the 'locobot/arm_base_link'
        self.bot.armtag.find_ref_to_arm_base_transform(position_only=True)
        # move the arm out of the way of the camera
        self.bot.arm.go_to_sleep_pose()

    def get_clusters(self):
        # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link';
        # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
        time.sleep(0.5)
        success, clusters = self.bot.pcl.get_cluster_positions(
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
        # move the robot back so it's centered and open the gripper
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
        self.bot.gripper.open()

        # pick up each object from left-to-right and drop it in a virtual basket on the left side of the robot
        for cluster in clusters:
            x, y, z = cluster["position"]
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
            self.bot.gripper.close()
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(y=0.3, z=0.2)
            self.bot.gripper.open()
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        self.bot.arm.go_to_sleep_pose()

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


