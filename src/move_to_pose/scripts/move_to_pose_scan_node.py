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
from src.move_to_pose_utils import load_poses, list_available_poses, create_goal

class LocobotMoveToPose:
    def __init__(self):
        rospy.init_node('locobot_move_to_pose')
        
        # Create action client
        self.client = actionlib.SimpleActionClient('/locobot/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for locobot move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Subscribe to costmap
        self.costmap = None
        self.costmap_sub = rospy.Subscriber(
            '/locobot/move_base/global_costmap/costmap',
            OccupancyGrid,
            self.costmap_callback
        )
        
        # Wait for first costmap
        rospy.loginfo("Waiting for costmap...")
        while self.costmap is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Received costmap")

        # Load poses from config
        self.poses_config = "/home/user/lips_ws/src/move_to_pose/config/poses.yaml"
        self.poses = load_poses(self.poses_config)
        
        if not self.poses:
            rospy.logerr("Failed to load poses from config!")
            sys.exit(1)

    def perform_scan_rotation(self):
        """Perform four 90-degree rotations in place"""
        rospy.loginfo("Starting scan rotation sequence")
        
        # Create a goal that maintains current position but changes orientation
        current_goal = MoveBaseGoal()
        current_goal.target_pose.header.frame_id = "map"
        current_goal.target_pose.header.stamp = rospy.Time.now()
        
        # Get current position (assuming we're at the intermediate point)
        current_pose = self.client.get_state()  # You might need to implement proper pose tracking
        
        # Perform 4 rotations (360 degrees total)
        for i in range(4):
            # Calculate next orientation (90-degree increments)
            yaw = math.pi / 2 * (i + 1)  # Convert to radians
            quaternion = quaternion_from_euler(0, 0, yaw)
            
            # Update goal orientation
            current_goal.target_pose.pose.orientation = Quaternion(*quaternion)
            
            # Send rotation goal
            rospy.loginfo(f"Executing rotation {i+1}/4 ({(i+1)*90} degrees)")
            self.client.send_goal(current_goal)
            self.client.wait_for_result()
            
            # Brief pause between rotations
            rospy.sleep(3.0)
        
        rospy.loginfo("Completed scan rotation sequence")

    def calculate_intermediate_point(self, start_x, start_y, end_x, end_y, ratio=0.7):
        """Calculate an intermediate point along the path"""
        # Simple linear interpolation
        int_x = start_x + (end_x - start_x) * ratio
        int_y = start_y + (end_y - start_y) * ratio
        return int_x, int_y

    def move_to_position(self, x, y, is_final=True):
        """Move to a specific position and perform scan if it's not the final destination"""
        if not self.is_position_safe(x, y):
            rospy.logerr(f"Position ({x}, {y}) is in unsafe area!")
            return False

        # Create and send goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        
        # Use default orientation (facing forward)
        quaternion = quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(*quaternion)

        rospy.loginfo(f"Moving to position: x={x:.2f}, y={y:.2f}")
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait or self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Failed to reach position")
            return False

        # If this is an intermediate point, perform the scan
        if not is_final:
            self.perform_scan_rotation()

        return True

    def move_to_named_pose(self, pose_name):
        """Move to a named pose with intermediate point and scanning"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return False

        pose_data = self.poses[pose_name]
        target_x = pose_data['position']['x']
        target_y = pose_data['position']['y']

        # Get current position (you might need to implement proper pose tracking)
        current_x = 0  # Replace with actual current position
        current_y = 0  # Replace with actual current position

        # Calculate intermediate point
        int_x, int_y = self.calculate_intermediate_point(
            current_x, current_y, target_x, target_y
        )

        # Move to intermediate point and perform scan
        if not self.move_to_position(int_x, int_y, is_final=False):
            return False

        # Move to final destination
        return self.move_to_position(target_x, target_y, is_final=True)

    def move_to_all_poses(self):
        """Move to each pose in sequence, with intermediate points and scanning"""
        for pose_name in self.poses.keys():
            rospy.loginfo(f"\nMoving to: {pose_name}")
            success = self.move_to_named_pose(pose_name)
            if not success:
                rospy.logwarn(f"Failed to reach {pose_name}, continuing to next pose...")
            rospy.sleep(1)

def main():
    try:
        mover = LocobotMoveToPose()
        args = [arg for arg in sys.argv[1:] if ':=' not in arg]
        
        if not args:
            mover.list_available_poses()
            rospy.spin()
            return

        command = args[0]
        
        if command == 'list':
            mover.list_available_poses()
        elif command == 'all':
            mover.move_to_all_poses()
        else:
            success = mover.move_to_named_pose(command)
            
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")

if __name__ == '__main__':
    main()