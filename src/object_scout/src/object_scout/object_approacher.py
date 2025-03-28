#!/usr/bin/env python3
import rospy
import math
import actionlib
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker



# ---------- CLASS DEFINITION ----------

class ObjectApproacher:
    """
    Handles approaching detected objects with ROS navigation
    
    This class provides:
    1. Object detection and depth monitoring
    2. Safe path planning to approach objects
    3. Graduated approach to target objects
    4. Marker position averaging for stable approaches
    """
    def __init__(self, robot_name, nav_controller, init_node=False):
        """
        Initialize the ObjectApproacher with robot configuration and ROS setup
        
        Args:
            robot_name (str): Name of the robot
            nav_controller: Controller for robot navigation
            init_node (bool): Whether to initialize a ROS node
        """
        # Initialize node if requested
        if init_node:
            rospy.init_node('object_approacher', anonymous=False)
            
        # ---------- CONFIGURATION ----------
            
        # Core components
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        self.costmap = nav_controller.costmap
        

        self.navigation_timeout = 40.0  # seconds
        
        # Marker history configuration
        self.max_history_length = 10     # Maximum positions to track
        self.min_positions_for_average = 3  # Minimum positions for reliable average
        
        # ---------- STATE VARIABLES ----------
        
        # Current sensing state
        self.current_depth = None
        self.object_marker = None
        self.bounding_box_coordinates = []
        
        # History tracking
        self.marker_positions = []
        
        # ---------- ROS INTERFACE SETUP ----------
        
        # Set up depth subscription
        self._setup_depth_subscription()
        
        # Set up object marker subscription
        self._setup_marker_subscription()

    # ---------- ROS COMMUNICATION SETUP ----------
    
    def _setup_depth_subscription(self):
        """Set up ROS subscription for depth information"""
        self.bbox_depth_topic = rospy.get_param(
            '~bbox_depth_topic', 
            f'/{self.robot_name}/camera/yolo/bbox_depth'
        )
        
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )
    
    def _setup_marker_subscription(self):
        """Set up ROS subscription for object marker information"""
        self.object_marker_topic = rospy.get_param(
            '~object_marker_topic', 
            f'/{self.robot_name}/object_markers'
        )
        
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )

    # ---------- CALLBACK METHODS ----------

    def object_marker_callback(self, msg):
        """
        Process object marker messages and update history
        
        Args:
            msg (Marker): Marker message from object detection
        """
        # Store the latest marker
        self.object_marker = msg

        # Add position to history if valid
        if msg is not None:
            position = (msg.pose.position.x, msg.pose.position.y)
            self.marker_positions.append(position)

            # Maintain history length
            if len(self.marker_positions) > self.max_history_length:
                self.marker_positions.pop(0)

    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg (Float32MultiArray): Message containing bounding box and depth data
                Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
        """

            
        # Extract values from new format
        n_boxes = int(msg.data[0])
        if n_boxes < 1:
            return
            
        cls_id = int(msg.data[1])
        confidence = float(msg.data[2])
        x1, y1 = float(msg.data[3]), float(msg.data[4])
        x2, y2 = float(msg.data[5]), float(msg.data[6])
        depth = float(msg.data[7])
        
        # Calculate width and height
        width = x2 - x1
        height = y2 - y1
        
        # Store bounding box coordinates [x, y, w, h]
        self.bounding_box_coordinates = [x1, y1, width, height]
        
        # Store depth value
        self.current_depth = depth
        
        # # Print depth information periodically
        # if hasattr(self, '_last_depth_print') and \
        # rospy.Time.now() - self._last_depth_print < rospy.Duration(1.0):
        #     # Skip printing too frequently
        #     pass
        # else:
        #     rospy.loginfo(f"Object depth: {self.current_depth:.2f}m, " + 
        #                 f"confidence: {confidence:.2f}, class: {cls_id}")
        #     self._last_depth_print = rospy.Time.now()

    # ---------- POSITION CALCULATION METHODS ----------

    def get_average_marker_position(self):
        """
        Calculate the average position from recent marker history
        
        Returns:
            tuple: (avg_x, avg_y) coordinates, or None if insufficient data
            
        Notes:
            - Uses a minimum number of positions for reliability
            - Calculates standard deviation for debugging if enough data
        """
        if len(self.marker_positions) < self.min_positions_for_average:
            rospy.logwarn(
                f"Insufficient marker positions for averaging: "
                f"{len(self.marker_positions)}/{self.min_positions_for_average}"
            )
            
            # Return the latest position if we have at least one
            if len(self.marker_positions) > 0:
                return self.marker_positions[-1]
            return None
        
        # Calculate average position
        avg_x = sum(pos[0] for pos in self.marker_positions) / len(self.marker_positions)
        avg_y = sum(pos[1] for pos in self.marker_positions) / len(self.marker_positions)
        
        # Calculate standard deviation for debugging (if enough data)
        if len(self.marker_positions) >= 5:
            std_dev_x = math.sqrt(
                sum((pos[0] - avg_x)**2 for pos in self.marker_positions) / 
                len(self.marker_positions)
            )
            std_dev_y = math.sqrt(
                sum((pos[1] - avg_y)**2 for pos in self.marker_positions) / 
                len(self.marker_positions)
            )
            rospy.logdebug(f"Position std dev: x={std_dev_x:.3f}m, y={std_dev_y:.3f}m")
        
        return avg_x, avg_y

    def _calculate_approach_waypoint(self, target_x, target_y, current_pose, retry=True):
        """
        Calculate the next waypoint for approaching the target
        
        Args:
            target_x (float): Target X coordinate
            target_y (float): Target Y coordinate
            current_pose: Current robot pose
            retry (bool): Whether to retry with a smaller step if first attempt fails
            
        Returns:
            tuple: (next_x, next_y) or (None, None) if no safe path found
        """
        # First attempt with normal step size
        next_x, next_y = self.nav_controller.calculate_safe_approach_point(
            target_x, target_y, current_pose, max_step=1.0
        )
        
        # Verify the calculated position is safe
        if not self.nav_controller.is_position_safe_approach(self.costmap, next_x, next_y):
            # If retry enabled, try with a shorter step
            if retry:
                next_x, next_y = self.nav_controller.calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                
                if not self.nav_controller.is_position_safe_approach(self.costmap, next_x, next_y):
                    rospy.logerr("Cannot find safe approach path!")
                    return None, None
            else:
                return None, None
                
        return next_x, next_y

    # ---------- VALIDATION METHODS ----------
    
    def _depth_and_marker_check(self, approach_max_depth, approach_min_depth):
        """
        Validate depth and marker with sustained detection for 1 second
        
        Returns:
            None: Continue approach
            True: Successfully reached target
            False: Detection failure
            
        Notes:
            - Requires sustained detection for reliability
            - Returns True when target depth range is reached
        """
        # Start time for sustained detection check
        start_time = rospy.Time.now()
        detection_duration = rospy.Duration(1.0)  # 1 second sustained detection
        
        # Rate for checking
        rate = rospy.Rate(10)  # 10 Hz
        
        # Monitor for sustained detection
        while (rospy.Time.now() - start_time) < detection_duration:
            # Debug logging
            rospy.logdebug(f"Depth check - Current depth: {self.current_depth}")
            rospy.logdebug(
                f"Object marker status: "
                f"{'Present' if self.object_marker is not None else 'None'}"
            )
            
            # Validate sensor data
            if self.current_depth is None:
                rospy.logwarn("No depth information available")
                return False
            elif self.object_marker is None:
                rospy.logwarn("Incomplete marker information")
                return False
            
            # Wait before next check
            rate.sleep()
        
        # Sustained detection successful
        rospy.loginfo(f"Sustained detection confirmed at depth: {self.current_depth:.2f}m")

        # Check if we've reached the target depth range
        if approach_min_depth <= self.current_depth <= approach_max_depth:   
            rospy.loginfo(f"Object detected at correct distance: {self.current_depth:.2f}m")

            # Cancel active navigation if needed
            if self.nav_controller.client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.nav_controller.cancel_navigation()
                rospy.sleep(0.5)
                
            return True
        else:
            # Object is detected but not at the correct distance yet
            rospy.loginfo(
                f"Current: {self.current_depth:.2f}m, "
                f"Target: {approach_min_depth:.2f}m-{approach_max_depth:.2f}m"
            )
            return None

    # ---------- NAVIGATION EXECUTION METHODS ----------
            
    def _execute_approach_step(self, next_x, next_y, target_x, target_y, current_pose):
        """
        Execute a single step in the approach sequence
        
        Args:
            next_x (float): Next waypoint X coordinate
            next_y (float): Next waypoint Y coordinate
            target_x (float): Target X coordinate
            target_y (float): Target Y coordinate
            current_pose: Current robot pose
            
        Returns:
            None: Continue approach
            True: Target reached
            False: Error occurred
        """
        # Calculate orientation to face the target
        dx = target_x - next_x
        dy = target_y - next_y
        angle = math.atan2(dy, dx)
        quaternion = quaternion_from_euler(0, 0, angle)
        
        # Create navigation goal with orientation facing the target
        goal = self.nav_controller.create_navigation_goal(
            next_x, 
            next_y,
            Quaternion(*quaternion)
        )
        
        # Send the goal
        self.nav_controller.client.send_goal(goal)
        
        # Set up monitoring
        timeout = rospy.Duration(self.navigation_timeout)
        rate = rospy.Rate(10)  # 10Hz checking rate
        start_time = rospy.Time.now()
        
        # Monitor navigation execution
        while not rospy.is_shutdown():
            # Check if we've reached target depth during movement
            if self._check_depth_during_movement():
                return True

            # Check if we've timed out
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Goal timeout reached")
                self.nav_controller.cancel_navigation()
                return False

            # Check goal status
            state = self.nav_controller.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                return None  # Continue to next approach step
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                rospy.logwarn("Goal aborted or rejected")
                return False

            rate.sleep()
            
        return False  # If we get here, we've been shutdown
    
    def _check_depth_during_movement(self):
        """
        Check if we've reached target depth during movement
        
        Returns:
            bool: True if target depth reached, False otherwise
        """
        if self.current_depth is not None and (
            self.approach_min_depth <= self.current_depth <= self.approach_max_depth):
            
            rospy.loginfo(f"Reached target depth during movement: {self.current_depth:.2f}m")
            self.nav_controller.cancel_navigation()
            rospy.sleep(0.5)  # Wait for cancellation to take effect
            
            # Verify cancellation state
            if self.nav_controller.client.get_state() in [
                actionlib.GoalStatus.PREEMPTED,
                actionlib.GoalStatus.RECALLED,
                actionlib.GoalStatus.ABORTED
            ]:
                return True
            else:
                rospy.logwarn(
                    f"Unexpected state after cancellation: "
                    f"{self.nav_controller.client.get_state()}"
                )
                return False
                
        return False

    # ---------- MAIN APPROACH METHOD ----------

    def approach_object(self, approach_max_depth=1.0, approach_min_depth=0.8):
        """
        Approach a detected object with graduated approach and marker tracking
        
        Args:
            object_marker (Marker): The marker for the detected object
            
        Returns:
            bool: Success flag
            
        Notes:
            - Uses marker position averaging for stable approaches
            - Monitors depth continuously during approach
            - Implements graduated approach with safe path planning
        """
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        # Initial validation
        if self.object_marker is None:
            rospy.logerr("No object marker available to approach, check topic")
            return False
        
        # Store initial marker details as a fallback
        initial_target_x = self.object_marker.pose.position.x
        initial_target_y = self.object_marker.pose.position.y

        # Initialize marker position history with initial position
        self.marker_positions = [(initial_target_x, initial_target_y)]
        
        # Get initial robot pose (for potential fallback)
        original_pose = self.nav_controller.get_robot_pose()
        if original_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        # Track state for logging
        approach_iterations = 0
        last_depth = None
        consecutive_detection_failures = 0

        # Main approach loop
        while not rospy.is_shutdown():
            # Increment approach iteration counter
            approach_iterations += 1
            
            # Log depth changes
            if self.current_depth is not None and last_depth != self.current_depth:
                rospy.loginfo(
                    f"Approach iteration {approach_iterations}: "
                    f"Current depth = {self.current_depth:.2f}m"
                )
                last_depth = self.current_depth

            # Get averaged marker position (or fallback to initial)
            avg_position = self.get_average_marker_position()
            if avg_position is not None:
                target_x, target_y = avg_position
                rospy.loginfo(f"Using averaged marker position: ({target_x:.2f}, {target_y:.2f})")
            else:
                # Use initial target if no average available
                target_x, target_y = initial_target_x, initial_target_y
                rospy.logwarn("Using initial marker position as fallback")

            # Perform depth and marker validation
            depth_check = self._depth_and_marker_check(approach_max_depth, approach_min_depth)
            
            # Process depth check results
            if depth_check is True:
                return True
            elif depth_check is False:
                return False
            # If None, continue approach

            # Get current pose
            current_pose = self.nav_controller.get_robot_pose()
            if current_pose is None:
                rospy.logerr("Failed to get current robot pose")
                return False
            
            # Calculate next approach waypoint
            next_x, next_y = self._calculate_approach_waypoint(target_x, target_y, current_pose)
            if next_x is None:
                rospy.logerr("Cannot find safe approach path!")
                return False
            
            # Execute approach step
            approach_result = self._execute_approach_step(
                next_x, next_y, target_x, target_y, current_pose
            )
            
            # Process approach step result
            if approach_result is False:
                consecutive_detection_failures += 1
                if consecutive_detection_failures >= 3:
                    rospy.logerr("Too many consecutive detection failures")
                    return False
            elif approach_result is True:
                rospy.loginfo("Successfully reached target")
                return True
            # If None, continue to next approach step
        
        return False  # If we get here, we've been shutdown
    

    # ---------- Object Saving -----------
    def save_object(self):
        """
        Save the pose of the detected object for later use"
        """

        if self.object_marker is None:
            rospy.logerr("No object marker available to save")
            return False

        # Extract position from the marker
        position = self.object_marker.pose.position
        rospy.loginfo(f"Saving object position: ({position.x}, {position.y}, {position.z})")
        
        # Here you would implement saving logic, e.g., to a database or file
        # For now, we just log the action
        return position

# ---------- DIRECT EXECUTION BLOCK ----------

if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        rospy.loginfo("ObjectApproacher cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object approacher interrupted")