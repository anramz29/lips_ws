#!/usr/bin/env python3

import rospy
import yaml
from move_base_msgs.msg import MoveBaseActionGoal
import os

class GoalRecorder:
    def __init__(self):
        rospy.init_node('goal_recorder')
        self.config_file = os.path.expanduser('/home/rosuser/catkin_ws/src/move_to_pose/config/poses.yaml')
        self.locations = self.load_existing_poses()
        self.goal_received = False
        self.location_name = self.get_location_name()
        
        rospy.Subscriber('/locobot/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        rospy.loginfo(f"Waiting for goal for location: {self.location_name}")

    def get_location_name(self):
        return input("Enter location name: ")

    def load_existing_poses(self):
        try:
            with open(self.config_file, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('locations', {}) if data else {}
        except (IOError, yaml.YAMLError):
            return {}

    def goal_callback(self, msg):
        if not self.goal_received:
            pose = msg.goal.target_pose.pose
            
            pose_dict = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                },
                'name': self.location_name
            }
            
            self.locations[self.location_name] = pose_dict
            self.save_to_yaml()
            rospy.loginfo(f"Saved goal position as {self.location_name}")
            
            # Ask if user wants to record another location
            if input("Record another location? (y/n): ").lower() == 'y':
                self.location_name = self.get_location_name()
                self.goal_received = False
            else:
                rospy.signal_shutdown("Done recording goals")

    def save_to_yaml(self):
        try:
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            with open(self.config_file, 'w') as f:
                yaml.dump({'locations': self.locations}, f, default_flow_style=False)
            rospy.loginfo(f"Successfully saved to {self.config_file}")
        except Exception as e:
            rospy.loginfo(f"Error saving file: {e}")

if __name__ == '__main__':
    try:
        recorder = GoalRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass