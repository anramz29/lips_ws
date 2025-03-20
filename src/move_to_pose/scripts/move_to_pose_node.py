#!/usr/bin/env python3
import rospy
import actionlib
import yaml
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import sys
from tf.transformations import euler_from_quaternion
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
        self.poses_config = "/home/rosuser/lips_ws/src/move_to_pose/config/poses.yaml"

        # Load poses from config
        self.poses = load_poses(self.poses_config)
        
        if not self.poses:
            rospy.logerr("Failed to load poses from config!")
            sys.exit(1)

    def costmap_callback(self, msg):
        self.costmap = msg

    def list_available_poses(self):
        # Call the utility function with the class's poses
        list_available_poses(self.poses)


    def is_position_safe(self, x, y):
        """Check if a position is in a safe area of the costmap"""
        if self.costmap is None:
            rospy.logwarn("No costmap available")
            return False

        # Convert world coordinates to costmap cell coordinates
        cell_x = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        cell_y = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)

        # Check if coordinates are within costmap bounds
        if (cell_x < 0 or cell_x >= self.costmap.info.width or
            cell_y < 0 or cell_y >= self.costmap.info.height):
            rospy.logwarn(f"Position ({x}, {y}) is outside costmap bounds")
            return False

        # Get cost value at position
        index = cell_y * self.costmap.info.width + cell_x
        cost = self.costmap.data[index]

        # Cost values: -1 = unknown, 0 = free, 1-99 = cost, 100 = occupied
        if cost == -1:
            rospy.logwarn(f"Position ({x}, {y}) is in unknown space")
            return False
        elif cost >= 90:  # You can adjust this threshold
            rospy.logwarn(f"Position ({x}, {y}) is too close to obstacles (cost: {cost})")
            return False
        
        return True

    def move_to_named_pose(self, pose_name):
        """Move to a named pose from the config"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return False

        pose_data = self.poses[pose_name]
        x = pose_data['position']['x']
        y = pose_data['position']['y']

        # Check if position is safe
        if not self.is_position_safe(x, y):
            rospy.logerr(f"Position for {pose_name} is in unsafe area!")
            return False

        goal = create_goal(pose_data)
        rospy.loginfo(f"Moving to location: {pose_data['name']}")
        
        # Send goal and wait for result
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            return False
        else:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
                return True
            else:
                rospy.logwarn("Goal failed with status: " + str(state))
                return False

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