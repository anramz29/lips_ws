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
        self.poses = self.load_poses()
        
        if not self.poses:
            rospy.logerr("Failed to load poses from config!")
            sys.exit(1)

    def costmap_callback(self, msg):
        self.costmap = msg

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

    def load_poses(self):
        """Load poses from YAML config file"""
        try:
            rospack = rospkg.RosPack()
            config_path = rospack.get_path('move_to_pose') + '/config/poses.yaml'
            with open(config_path, 'r') as file:
                return yaml.safe_load(file)['locations']
        except Exception as e:
            rospy.logerr(f"Error loading config: {str(e)}")
            return None

    def create_goal(self, pose_data):
        """Create a MoveBaseGoal from pose data"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set position
        goal.target_pose.pose.position = Point(
            pose_data['position']['x'],
            pose_data['position']['y'],
            pose_data['position']['z']
        )

        # Set orientation
        goal.target_pose.pose.orientation = Quaternion(
            pose_data['orientation']['x'],
            pose_data['orientation']['y'],
            pose_data['orientation']['z'],
            pose_data['orientation']['w']
        )

        return goal

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

        goal = self.create_goal(pose_data)
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

    def list_available_poses(poses):
        """List all available poses and their coordinates"""
        if not poses:
            rospy.logwarn("No poses available - check if poses.yaml is loaded correctly")
            return

        rospy.loginfo("\nAvailable poses:")
        for name, pose_data in poses.items():
            pos = pose_data['position']
            rospy.loginfo(f"- {name}: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}")


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