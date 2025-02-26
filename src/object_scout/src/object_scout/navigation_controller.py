#!/usr/bin/env python3
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalID
import actionlib_msgs.msg

# Import utilities
from object_scout.utils import get_robot_pose, is_position_safe


class NavigationController:
    """
    Handles basic robot navigation operations and interfaces with move_base
    """
    def __init__(self, robot_name, init_node=False):
        """
        Initialize the navigation controller
        
        Args:
            robot_name: Name of the robot for topic namespacing
            init_node: Whether to initialize a ROS node (standalone mode)
        """
        if init_node:
            rospy.init_node('navigation_controller', anonymous=False)
        
        self.robot_name = robot_name
        
        # Get topic parameters with default values that use robot_name
        self.move_base_topic = rospy.get_param('~move_base_topic', f'/{self.robot_name}/move_base')
        self.costmap_topic = rospy.get_param('~costmap_topic', f'/{self.robot_name}/move_base/global_costmap/costmap')
        
        # Create action client
        self.client = actionlib.SimpleActionClient(self.move_base_topic, MoveBaseAction)
        rospy.loginfo(f"Waiting for {self.robot_name} move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Setup cancel publisher
        self.cancel_pub = rospy.Publisher(
            f'{self.move_base_topic}/cancel', 
            actionlib_msgs.msg.GoalID, 
            queue_size=1
        )
        
        # Subscribe to costmap
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
        
    def costmap_callback(self, msg):
        """Callback for costmap updates"""
        self.costmap = msg
        
    def move_to_position(self, x, y, orientation=None, timeout=30.0):
        """
        Move to a specific position with optional orientation
        
        Args:
            x: X coordinate in the map frame
            y: Y coordinate in the map frame
            orientation: Optional Quaternion orientation
            timeout: Navigation timeout in seconds
            
        Returns:
            bool: Success flag
        """
        # Safety check
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False
            
        if not is_position_safe(self.costmap, x, y):
            rospy.logerr(f"Position ({x}, {y}) is in unsafe area!")
            return False

        # Create navigation goal
        goal = self.create_navigation_goal(x, y, orientation, current_pose)
        
        # Send the goal
        rospy.loginfo(f"Moving to position: x={x:.2f}, y={y:.2f}")
        self.client.send_goal(goal)
        
        # Wait for result with timeout
        return self.wait_for_navigation(timeout)
        
    def wait_for_navigation(self, timeout):
        """
        Wait for navigation to complete with timeout
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            bool: Success flag
        """
        wait = self.client.wait_for_result(rospy.Duration(timeout))
        
        if not wait:
            rospy.logerr(f"Navigation timed out after {timeout} seconds")
            self.client.cancel_goal()
            return False
            
        success = self.client.get_state() == actionlib.GoalStatus.SUCCEEDED
        
        if not success:
            state = self.client.get_state()
            state_txt = actionlib.GoalStatus.to_string(state) if state <= 9 else "UNKNOWN"
            rospy.logerr(f"Navigation failed with state: {state_txt} ({state})")
            
        return success
        
    def cancel_navigation(self):
        """Cancel current navigation goal using both action client and topic"""
        # Cancel via action client
        self.client.cancel_all_goals()
        # Cancel via topic for redundancy
        cancel_msg = actionlib_msgs.msg.GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Sent navigation cancellation commands")
        
    def create_navigation_goal(self, x, y, orientation=None, current_pose=None):
        """
        Create a MoveBaseGoal message for the specified position and orientation
        
        Args:
            x: X coordinate in the map frame
            y: Y coordinate in the map frame
            orientation: Optional Quaternion orientation
            current_pose: Current robot pose for calculating direction
            
        Returns:
            MoveBaseGoal: The configured goal message
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        
        # Handle orientation
        if orientation:
            # Validate the quaternion
            norm = math.sqrt(
                orientation.x**2 + orientation.y**2 + 
                orientation.z**2 + orientation.w**2
            )
            
            # If the quaternion is nearly zero length or very far from unit length
            if norm < 0.1:
                rospy.logwarn(f"Quaternion has length close to zero: {norm}, using default")
                # Use default orientation instead
                if current_pose:
                    # Calculate angle to target
                    dx = x - current_pose.position.x
                    dy = y - current_pose.position.y
                    angle = math.atan2(dy, dx)
                    quat = quaternion_from_euler(0, 0, angle)
                    goal.target_pose.pose.orientation = Quaternion(*quat)
                else:
                    # Default facing forward
                    quat = quaternion_from_euler(0, 0, 0)
                    goal.target_pose.pose.orientation = Quaternion(*quat)
            elif abs(norm - 1.0) > 0.01:
                # Normalize the quaternion if it's not unit length
                rospy.logwarn(f"Quaternion not normalized (norm={norm}), normalizing")
                factor = 1.0 / norm
                normalized_quat = Quaternion(
                    orientation.x * factor,
                    orientation.y * factor,
                    orientation.z * factor,
                    orientation.w * factor
                )
                goal.target_pose.pose.orientation = normalized_quat
            else:
                # Use the provided orientation as is
                goal.target_pose.pose.orientation = orientation
        else:
            # No orientation provided, calculate one
            if current_pose:
                # Face the direction of movement
                dx = x - current_pose.position.x
                dy = y - current_pose.position.y
                angle = math.atan2(dy, dx)
                quat = quaternion_from_euler(0, 0, angle)
                rospy.loginfo(f"Setting orientation to face direction of travel (yaw: {math.degrees(angle):.2f} degrees)")
            else:
                # Default facing forward (positive x-axis)
                quat = quaternion_from_euler(0, 0, 0)
                
            goal.target_pose.pose.orientation = Quaternion(*quat)
            
        return goal


if __name__ == "__main__":
    try:
        # Run as standalone node
        controller = NavigationController(
            robot_name=rospy.get_param('~robot_name', 'locobot'),
            init_node=True
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation controller interrupted")