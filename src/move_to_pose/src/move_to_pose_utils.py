import rospy
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import yaml
import tf2_ros
from geometry_msgs.msg import PoseStamped
import actionlib



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
    
def get_robot_pose():
    """Get current robot pose in map frame"""
    try:
        # Create tf buffer and listener when needed
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        # Give time for the listener to receive transforms
        rospy.sleep(0.5)
        
        trans = tf_buffer.lookup_transform('map', 'locobot/base_link', rospy.Time(0))
        current_pose = PoseStamped()
        current_pose.pose.position.x = trans.transform.translation.x
        current_pose.pose.position.y = trans.transform.translation.y
        current_pose.pose.position.z = trans.transform.translation.z
        current_pose.pose.orientation = trans.transform.rotation
        return current_pose.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Failed to get robot pose: {e}")
        return None

def list_available_poses(poses):
    """List all available poses and their coordinates"""
    if not poses:
        rospy.logwarn("No poses available - check if poses.yaml is loaded correctly")
        return

    rospy.loginfo("\nAvailable poses:")
    for name, pose_data in poses.items():
        pos = pose_data['position']
        rospy.loginfo(f"- {name}: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}")

