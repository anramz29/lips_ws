#!/usr/bin/env python3
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
import actionlib_msgs.msg
import tf2_ros
from geometry_msgs.msg import PoseStamped
import std_srvs.srv


class NavigationController:
    """
    Handles robot navigation operations and interfaces with move_base
    
    This class provides functionality to:
    1. Create and send navigation goals
    2. Monitor navigation progress
    3. Ensure safe navigation using costmaps
    4. Navigate to named poses and specific coordinates
    5. Handle orientation calculations and normalization
    """
    def __init__(self, robot_name, pose_manager, init_node=False):
        """
        Initialize the navigation controller
        
        Args:
            robot_name (str): Name of the robot for topic namespacing
            pose_manager: Optional PoseManager instance for named poses
            init_node (bool): Whether to initialize a ROS node (standalone mode)
        """
        if init_node:
            rospy.init_node('navigation_controller', anonymous=False)
        
        # ---------- CONFIGURATION ----------
        
        self.robot_name = robot_name
        self.pose_manager = pose_manager
        
        # Get topic parameters with default values that use robot_name
        self.move_base_topic = rospy.get_param('~move_base_topic', f'/{self.robot_name}/move_base')
        self.costmap_topic = rospy.get_param('~costmap_topic', f'/{self.robot_name}/move_base/global_costmap/costmap')
        
        # Default timeout values
        self.default_navigation_timeout = 90.0  # seconds

        
        # ---------- ROS INTERFACE SETUP ----------
        
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


        
    # ---------- CALLBACK METHODS ----------
    
    def costmap_callback(self, msg):
        """
        Callback for costmap updates
        
        Args:
            msg (OccupancyGrid): The costmap message
        """
        self.costmap = msg
    
    # ---------- GOAL CREATION METHODS ----------
    
    def create_navigation_goal(self, x, y, orientation=None, current_pose=None):
        """
        Create a MoveBaseGoal message for the specified position and orientation
        
        Args:
            x (float): X coordinate in the map frame
            y (float): Y coordinate in the map frame
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
    
    # ---------- NAVIGATION EXECUTION METHODS ----------
    
    def move_to_position(self, x, y, orientation=None, timeout=None):
        """
        Move to a specific position with optional orientation
        
        Args:
            x (float): X coordinate in the map frame
            y (float): Y coordinate in the map frame
            orientation: Optional Quaternion orientation
            timeout (float): Navigation timeout in seconds (default: self.default_navigation_timeout)
            
        Returns:
            bool: Success flag
        """
        # Use default timeout if not specified
        if timeout is None:
            timeout = self.default_navigation_timeout
        
        # Safety check
        current_pose = self.get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False
            
        if not self.is_position_safe(self.costmap, x, y):
            rospy.logerr(f"Position ({x:.2f}, {y:.2f}) is in unsafe area!")
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
            timeout (float): Timeout in seconds
            
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
        """
        Cancel current navigation goal using both action client and topic
        
        Uses redundant cancellation methods to ensure the goal is properly canceled.
        """
        # Cancel via action client
        self.client.cancel_all_goals()
        # Cancel via topic for redundancy
        cancel_msg = actionlib_msgs.msg.GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Sent navigation cancellation commands")

    def get_robot_pose_postions(self):
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
            return current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get robot pose: {e}")
            return None
    
    # ---------- NAMED POSE NAVIGATION METHODS ----------
    
    def navigate_to_named_pose(self, pose_name, timeout=None):
        """
        Navigate the robot to a named pose from the pose manager
        
        Args:
            pose_name (str): Name of the pose to navigate to
            timeout (float): Navigation timeout in seconds (default: self.default_navigation_timeout)
            
        Returns:
            bool: True if navigation succeeded, False otherwise
            
        Raises:
            ValueError: If pose_manager is not set or pose_name not found
        """
        if self.pose_manager is None:
            raise ValueError("PoseManager not set in NavigationController")
        
        rospy.loginfo(f"Navigating to named pose: {pose_name}")
        
        try:
            # Get pose from the pose manager
            pose_position = self.pose_manager.get_pose(pose_name)
            
            if pose_position is None:
                rospy.logwarn(f"Pose '{pose_name}' not found in pose manager")
                return False
                
            # Extract position
            x = pose_position.position.x
            y = pose_position.position.y
            
            # Extract orientation if available
            orientation = None
            if hasattr(pose_position, 'orientation'):
                orientation = pose_position.orientation
            
            # Execute navigation
            success = self.move_to_position(x, y, orientation, timeout)
            
            if not success:
                rospy.logwarn(f"Failed to navigate to pose '{pose_name}'")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"Error navigating to pose '{pose_name}': {e}")
            return False
    
    def return_to_position(self, x, y, orientation=None, timeout=None):
        """
        Return the robot to a specified position and orientation
        
        This method is semantically different from move_to_position as it indicates
        an intent to return to a previously visited position.
        
        Args:
            x (float): X coordinate to return to
            y (float): Y coordinate to return to
            orientation: Orientation to set at the position
            timeout (float): Navigation timeout in seconds (default: self.default_navigation_timeout)
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        rospy.loginfo(f"Returning to position: ({x:.2f}, {y:.2f})")
        return self.move_to_position(x, y, orientation, timeout)
    
    def rotate_in_place(self, angle_degrees, timeout=None):
        """
        Rotate the robot in place by the specified angle
        
        Args:
            angle_degrees (float): Angle to rotate in degrees
            timeout (float): Rotation timeout in seconds (default: 10.0)
            
        Returns:
            bool: True if rotation succeeded, False otherwise
        """
        if timeout is None:
            timeout = 10.0  # Default timeout for rotation (shorter than navigation)
        
        # Get current pose
        current_pose = self.get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False
        
        # Convert angle to radians
        angle_radians = math.radians(angle_degrees)
        
        # Create quaternion for the target orientation
        quat = quaternion_from_euler(0, 0, angle_radians)
        orientation = Quaternion(*quat)
        
        # Create goal at current position with new orientation
        rospy.loginfo(f"Rotating {angle_degrees} degrees in place")
        goal = self.create_navigation_goal(
            current_pose.position.x,
            current_pose.position.y,
            orientation
        )
        
        # Send the goal
        self.client.send_goal(goal)
        
        # Wait for result with timeout
        return self.wait_for_navigation(timeout)
    
    def get_robot_pose(self):
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
        
    def is_position_safe(self, costmap, x, y):
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
    
    def calculate_safe_approach_point(self, target_x, target_y, current_pose, max_step=1.0, target_distance=0.9):
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
    
    def get_adaptive_cost_threshold(self, distance):
        """Get cost threshold that becomes more lenient with distance"""
        # Base threshold for close distances
        base_threshold = 90
        # Reduce threshold (become more lenient) as distance increases
        distance_factor = min(distance / 2.0, 1.0)  # Cap at 2 meters
        return base_threshold * (1.0 - distance_factor * 0.3)  # Can go down to 70% of base threshold

    def calculate_intermediate_point(self, start_pose, end_pose, ratio=0.5):
        """Calculate the midpoint between start and end pose"""
        int_x = start_pose.position.x + (end_pose.position.x - start_pose.position.x) * ratio
        int_y = start_pose.position.y + (end_pose.position.y - start_pose.position.y) * ratio
        return int_x, int_y

    def is_position_safe_approach(self, costmap, x, y, current_pose=None):
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
            threshold = self.get_adaptive_cost_threshold(distance)
        else:
            threshold = 90  # Default threshold

        if cost == -1:
            rospy.logwarn(f"Position ({x}, {y}) is in unknown space")
            return False
        elif cost >= threshold:
            rospy.logwarn(f"Position ({x}, {y}) is too close to obstacles (cost: {cost}, threshold: {threshold})")
            return False
        
        return True
    
    def clear_costmaps(self):
        """
        Clear both local and global costmaps
        
        This is useful when:
        - The robot gets stuck due to phantom obstacles
        - After an environment change that should clear previously detected obstacles
        - Before starting a new navigation task
        
        Returns:
            bool: Success flag
        """
        try:
            # Service name for clearing costmaps
            clear_costmaps_service = f'{self.move_base_topic}/clear_costmaps'
            
            # Wait for service to be available
            rospy.loginfo(f"Waiting for {clear_costmaps_service} service...")
            rospy.wait_for_service(clear_costmaps_service, timeout=5.0)
            
            # Call the service
            clear_costmaps = rospy.ServiceProxy(clear_costmaps_service, std_srvs.srv.Empty)
            clear_costmaps()
            
            rospy.loginfo("Successfully cleared costmaps")
            return True
            
        except rospy.ROSException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear costmaps: {e}")
            return False


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