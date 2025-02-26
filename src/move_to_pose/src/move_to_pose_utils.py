import rospy
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import yaml
import tf2_ros
from geometry_msgs.msg import PoseStamped
import actionlib
import math
from tf.transformations import quaternion_from_euler


def create_goal(pose_data):
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

def load_poses(poses_config):
    """Load poses from YAML config file"""
    try:
        with open(poses_config, 'r') as file:
            return yaml.safe_load(file)['locations']
    except Exception as e:
        rospy.logerr(f"Error loading config: {str(e)}")
        return None
    
def list_available_poses(poses):
    """
    List all available poses and their coordinates
    
    Args:
        poses (dict): Dictionary of poses
    """
    if not poses or not isinstance(poses, dict):
        rospy.logwarn("No poses available or invalid format - check if poses.yaml is loaded correctly")
        return
        
    rospy.loginfo("\nAvailable poses:")
    for name, pose_data in poses.items():
        if isinstance(pose_data, dict) and 'position' in pose_data:
            pos = pose_data['position']
            rospy.loginfo(f"- {name}: x={pos.get('x', 0):.2f}, y={pos.get('y', 0):.2f}, z={pos.get('z', 0):.2f}")
        else:
            rospy.logwarn(f"Invalid pose data format for {name}")

