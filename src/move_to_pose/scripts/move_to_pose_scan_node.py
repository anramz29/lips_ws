#!/usr/bin/env python3
import rospy
import actionlib
import yaml
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.move_to_pose_utils import load_poses, is_position_safe, get_robot_pose

class Move_and_Scan:
    def __init__(self):
        rospy.init_node('move_to_pose_scan_node', anonymous=True)
        
        # Get ROS parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '/home/rosuser/lips_ws/src/move_to_pose/config/poses.yaml')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        
        # Get topic parameters with default values that use robot_name
        self.move_base_topic = rospy.get_param('~move_base_topic', f'/{self.robot_name}/move_base')
        self.costmap_topic = rospy.get_param('~costmap_topic', f'/{self.robot_name}/move_base/global_costmap/costmap')
        
        # Create action client using the configured move_base topic
        action_server = self.move_base_topic
        self.client = actionlib.SimpleActionClient(action_server, MoveBaseAction)
        rospy.loginfo(f"Waiting for {self.robot_name} move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Subscribe to costmap using the configured topic
        self.costmap = None
        self.costmap_sub = rospy.Subscriber(
            self.costmap_topic,
            OccupancyGrid,
            self.costmap_callback
        )
        
        # Wait for first costmap
        rospy.loginfo("Waiting for costmap...")
        while self.costmap is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Received costmap")

        # Load poses from config
        self.poses = load_poses(self.poses_config)
        
        if not self.poses:
            rospy.logerr(f"Failed to load poses from config file: {self.poses_config}")
            sys.exit(1)
            
        rospy.loginfo(f"Loaded {len(self.poses)} poses from {self.poses_config}")
        rospy.loginfo(f"Pose command: {self.pose_command}")

    def costmap_callback(self, msg):
        self.costmap = msg

    def perform_scan_rotation(self):
        """Perform four 90-degree rotations in place"""
        rospy.loginfo("Starting scan rotation sequence")
        
        # Get current position
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return
        
        # Create a goal that maintains current position but changes orientation
        current_goal = MoveBaseGoal()
        current_goal.target_pose.header.frame_id = "map"
        current_goal.target_pose.pose.position = Point(
            current_pose.position.x,
            current_pose.position.y,
            0
        )
        
        # Perform 4 rotations (360 degrees total)
        for i in range(4):
            # Calculate next orientation (90-degree increments)
            yaw = math.pi / 2 * (i + 1)  # Convert to radians
            quaternion = quaternion_from_euler(0, 0, yaw)
            
            # Update goal orientation
            current_goal.target_pose.header.stamp = rospy.Time.now()
            current_goal.target_pose.pose.orientation = Quaternion(*quaternion)
            
            # Send rotation goal
            rospy.loginfo(f"Executing rotation {i+1}/4 ({(i+1)*90} degrees)")
            self.client.send_goal(current_goal)
            self.client.wait_for_result()
            
            # Brief pause between rotations
            rospy.sleep(2.0)
        
        rospy.loginfo("Completed scan rotation sequence")

    def calculate_intermediate_point(self, start_pose, end_pose, ratio=0.5):
        """Calculate the midpoint between start and end pose"""
        int_x = start_pose.position.x + (end_pose.position.x - start_pose.position.x) * ratio
        int_y = start_pose.position.y + (end_pose.position.y - start_pose.position.y) * ratio
        return int_x, int_y

    def move_to_position(self, x, y, orientation=None, scan_at_goal=False):
        """Move to a specific position with optional orientation"""
        if not is_position_safe(self.costmap, x, y):
            rospy.logerr(f"Position ({x}, {y}) is in unsafe area!")
            return False

        # Create and send goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        
        # Use provided orientation or default
        if orientation:
            goal.target_pose.pose.orientation = orientation
        else:
            quaternion = quaternion_from_euler(0, 0, 0)
            goal.target_pose.pose.orientation = Quaternion(*quaternion)

        rospy.loginfo(f"Moving to position: x={x:.2f}, y={y:.2f}")
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        success = wait and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED
        
        # Perform a scan if requested and movement was successful
        if success and scan_at_goal:
            rospy.loginfo("Performing scan at goal position")
            self.perform_scan_rotation()
            
        return success

    def move_to_named_pose(self, pose_name):
        """Move to a named pose with a single intermediate scanning point"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return False

        # Get current robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False

        # Get target pose
        target_pose = Pose()
        target_pose.position.x = self.poses[pose_name]['position']['x']
        target_pose.position.y = self.poses[pose_name]['position']['y']
        
        # Calculate halfway point
        int_x, int_y = self.calculate_intermediate_point(current_pose, target_pose)
        
        # Move to intermediate point
        rospy.loginfo(f"Moving to intermediate point: x={int_x:.2f}, y={int_y:.2f}")
        if not self.move_to_position(int_x, int_y):
            rospy.logerr("Failed to reach intermediate point")
            return False
            
        # Perform scan at intermediate point
        self.perform_scan_rotation()
        
        # Move to final destination and scan there
        rospy.loginfo(f"Moving to final position: {pose_name}")
        return self.move_to_position(
            target_pose.position.x,
            target_pose.position.y,
            scan_at_goal=True  # Perform scan at the final destination
        )

    def move_to_all_poses(self):
        """Move to each pose in sequence"""
        for pose_name in self.poses.keys():
            rospy.loginfo(f"\nMoving to: {pose_name}")
            success = self.move_to_named_pose(pose_name)
            if not success:
                rospy.logwarn(f"Failed to reach {pose_name}, continuing to next pose...")
            rospy.sleep(1)  # Brief pause between movements

def main():
    try:
        mover = Move_and_Scan()
        mover.move_to_all_poses()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")

if __name__ == '__main__':
    main()