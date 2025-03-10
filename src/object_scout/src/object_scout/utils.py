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

def is_position_safe(costmap, x, y):
    """Check if a position is in a safe area of the costmap"""
    if costmap is None:
        rospy.logwarn("No costmap available")
        return False

    # Convert world coordinates to costmap cell coordinates
    cell_x = int((x - costmap.info.origin.position.x) / costmap.info.resolution)
    cell_y = int((y - costmap.info.origin.position.y) / costmap.info.resolution)

    # Check if coordinates are within costmap bounds
    if (cell_x < 0 or cell_x >= costmap.info.width or
        cell_y < 0 or cell_y >= costmap.info.height):
        rospy.logwarn(f"Position ({x}, {y}) is outside costmap bounds")
        return False

    # Get cost value at position
    index = cell_y * costmap.info.width + cell_x
    cost = costmap.data[index]

    # Cost values: -1 = unknown, 0 = free, 1-99 = cost, 100 = occupied
    if cost == -1:
        rospy.logwarn(f"Position ({x}, {y}) is in unknown space")
        return False
    elif cost >= 90:  # You can adjust this threshold
        rospy.logwarn(f"Position ({x}, {y}) is too close to obstacles (cost: {cost})")
        return False
    
    return True
def calculate_safe_approach_point(target_x, target_y, current_pose, max_step=1.0, target_distance=0.9):
    """
    Calculate a safe intermediate point towards the target at a desired distance
    
    Args:
        target_x: Target X coordinate
        target_y: Target Y coordinate  
        current_pose: Current robot pose
        max_step: Maximum step size to take
        target_distance: Desired final distance from target (defaults to 1.1m, middle of 1.0-1.2m range)
        
    Returns:
        (next_x, next_y): Coordinates for the next waypoint
    """
    dx = target_x - current_pose.position.x
    dy = target_y - current_pose.position.y
    current_target_distance = math.sqrt(dx*dx + dy*dy)
    
    # Calculate unit direction vector
    direction_x = dx / current_target_distance if current_target_distance > 0 else 0
    direction_y = dy / current_target_distance if current_target_distance > 0 else 0
    
    # If we're already very close to the target distance, make small adjustments
    if abs(current_target_distance - target_distance) < 0.1:
        # Make a small adjustment to get closer to exact target distance
        adjustment = (target_distance - current_target_distance) * 0.5  # 50% adjustment
        return (
            current_pose.position.x + direction_x * adjustment,
            current_pose.position.y + direction_y * adjustment
        )
    
    # If we're too close to the target, back up
    if current_target_distance < target_distance:
        # Calculate point that's at target_distance away from target (back up)
        return (
            target_x - (direction_x * target_distance),
            target_y - (direction_y * target_distance)
        )
    
    # If we're far away, take a step toward the target
    step_size = min(max_step, current_target_distance - target_distance)
    return (
        current_pose.position.x + direction_x * step_size,
        current_pose.position.y + direction_y * step_size
    )


def get_adaptive_cost_threshold(distance):
    """Get cost threshold that becomes more lenient with distance"""
    # Base threshold for close distances
    base_threshold = 90
    # Reduce threshold (become more lenient) as distance increases
    distance_factor = min(distance / 2.0, 1.0)  # Cap at 2 meters
    return base_threshold * (1.0 - distance_factor * 0.3)  # Can go down to 70% of base threshold

def calculate_intermediate_point(start_pose, end_pose, ratio=0.5):
    """Calculate the midpoint between start and end pose"""
    int_x = start_pose.position.x + (end_pose.position.x - start_pose.position.x) * ratio
    int_y = start_pose.position.y + (end_pose.position.y - start_pose.position.y) * ratio
    return int_x, int_y

def is_position_safe_approach(costmap, x, y, current_pose=None):
    """Enhanced safety check with distance-based threshold"""
    if costmap is None:
        rospy.logwarn("No costmap available")
        return False

    # Convert world coordinates to costmap cell coordinates
    cell_x = int((x - costmap.info.origin.position.x) / costmap.info.resolution)
    cell_y = int((y - costmap.info.origin.position.y) / costmap.info.resolution)

    # Check if coordinates are within costmap bounds
    if (cell_x < 0 or cell_x >= costmap.info.width or
        cell_y < 0 or cell_y >= costmap.info.height):
        rospy.logwarn(f"Position ({x}, {y}) is outside costmap bounds")
        return False

    # Get cost value at position
    index = cell_y * costmap.info.width + cell_x
    cost = costmap.data[index]

    # If we have current pose, use distance-based threshold
    if current_pose is not None:
        distance = math.sqrt(
            (x - current_pose.position.x)**2 + 
            (y - current_pose.position.y)**2
        )
        threshold = get_adaptive_cost_threshold(distance)
    else:
        threshold = 90  # Default threshold

    if cost == -1:
        rospy.logwarn(f"Position ({x}, {y}) is in unknown space")
        return False
    elif cost >= threshold:
        rospy.logwarn(f"Position ({x}, {y}) is too close to obstacles (cost: {cost}, threshold: {threshold})")
        return False
    
    return True
    