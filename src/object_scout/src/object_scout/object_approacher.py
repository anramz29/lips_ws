#!/usr/bin/env python3
import rospy
import math
import actionlib
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker

# Import utilities
from object_scout.utils import get_robot_pose, calculate_safe_approach_point, is_position_safe_approach


class ObjectApproacher:
    """
    Handles approaching detected objects
    """
    def __init__(self, robot_name, nav_controller, init_node=False):
        if init_node:
            rospy.init_node('object_approacher', anonymous=False)
            
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        self.costmap = nav_controller.costmap
        
        # Approacher state
        self.current_depth = None
        
        # Approach parameters
        self.approach_min_depth = .80  # meters
        self.approach_max_depth = 1 # meters
        self.navigation_timeout = 40.0  # seconds
        
        # Set up depth subscription
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 
                                               f'/{robot_name}/camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )

            # Set up object marker subscription
        self.object_marker_topic = rospy.get_param('~object_marker_topic', 
                                                f'/{robot_name}/object_markers')
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,  # Assuming this is the correct message type
            self.object_marker_callback
        )

        self.object_marker = None    # Set up object marker subscription
        self.bounding_box_cordinates = []

        # Add marker position history buffer
        self.marker_positions = []
        self.max_history_length = 10  # Adjust as needed
        self.min_positions_for_average = 3 # Minimum positions to calculate average


    def object_marker_callback(self, msg):
        """
        Process object marker messages
        
        Args:
            msg: Marker message from object detection
        """
        # Store the latest marker for approach
        self.object_marker = msg

        # Add Position to history
        if msg is not None:
            position = (msg.pose.position.x, msg.pose.position.y)
            self.marker_positions.append(position)

            if len(self.marker_positions) > self.max_history_length:
                self.marker_positions.pop(0)


    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg: Float32MultiArray containing depth data
        """
        if msg.data and len(msg.data) >= 5:
            # obtain bounding box coordinates
            self.bounding_box_cordinates = msg.data[:4]

            # Store the latest depth for approach
            self.current_depth = msg.data[4]
        else:
            rospy.logwarn("Invalid depth data received")

    def approach_object(self, object_marker):
        """
        Approach a detected object with improved marker persistence
        
        Args:
            object_marker: The marker for the detected object
            
        Returns:
            bool: Success flag
        """
        rospy.loginfo("Moving to detected object with graduated approach...")
        

        if object_marker is None:
            rospy.logerr("No object marker available to approach, check topic")
            return False
        
        # Store initial marker details as a fallback
        initial_target_x = object_marker.pose.position.x
        initial_target_y = object_marker.pose.position.y

        # Initialize marker position history with initial position
        self.marker_positions = [(initial_target_x, initial_target_y)]
        
        # Get initial robot pose (save for potential fallback)
        original_pose = get_robot_pose()
        if original_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        # Track detection reliability
        consecutive_detection_failures = 0

        # Approach loop
        approach_iterations = 0

        # last depth
        last_depth = None

        # Approach loop
        while not rospy.is_shutdown():

            # Increment approach iteration
            approach_iterations += 1
            
            # Log for debugging
            if self.current_depth is not None:
                if last_depth != self.current_depth:
                    rospy.loginfo(f"Approach iteration {approach_iterations}: Current depth = {self.current_depth:.2f}m")
                    last_depth = self.current_depth


            # Get average marker position or use fallback
            avg_position = self.get_average_marker_position()
            if avg_position is not None:
                target_x, target_y = avg_position
                rospy.loginfo(f"Using averaged marker position: ({target_x:.2f}, {target_y:.2f})")
            else:
                # Use initial target if no average available
                target_x, target_y = initial_target_x, initial_target_y
                rospy.logwarn("Using initial marker position as fallback")

            
            # Depth and marker validation
            depth_check = self._depth_and_marker_check()
            
            # if depth_check is None, continue to next iteration
            if depth_check is True:
                return True
            elif depth_check is False:
                return False

            # Calculate next approach waypoint
            current_pose = get_robot_pose()
            if current_pose is None:
                rospy.logerr("Failed to get current robot pose")
                return False
            
            next_x, next_y = self._calculate_approach_waypoint(target_x, target_y, current_pose)
            if next_x is None:
                rospy.logerr("Cannot find safe approach path!")
                return False
            
            # Execute approach step
            approach_result = self._execute_approach_step(
                next_x, next_y, target_x, target_y, current_pose
            )

            
            # Interpret approach result
            if approach_result is False:
                consecutive_detection_failures += 1
            elif approach_result is True:
                rospy.loginfo("Successfully reached target")
                return True
        
        return False

    def _depth_and_marker_check(self):
        """
        depth and marker validation with sustained detection for 1 second
        
        Args:
            initial_x: Initial target X coordinate
            initial_y: Initial target Y coordinate
        
        Returns:
            None: Continue approach
            True: Successfully reached target
            False: Detection failure
        """
        # Start time for sustained detection check
        start_time = rospy.Time.now()
        detection_duration = rospy.Duration(1.0)  # 1 second sustained detection
        
        # Rate for checking
        rate = rospy.Rate(10)  # 10 Hz
        
        while (rospy.Time.now() - start_time) < detection_duration:
            # Comprehensive logging
            rospy.logdebug(f"Depth check - Current depth: {self.current_depth}")
            rospy.logdebug(f"Object marker status: {'Present' if self.object_marker is not None else 'None'}")
            
            # Depth information check
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

        if self.approach_min_depth <= self.current_depth <= self.approach_max_depth:   
            rospy.loginfo(f"Object detected at correct distance: {self.current_depth:.2f}m")

            # Cancel active navigation if needed
            if self.nav_controller.client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.nav_controller.cancel_navigation()
                rospy.sleep(0.5)
                
            return True
        else:
            # Object is detected but not at the correct distance yet
            rospy.loginfo(f" Current:  {self.current_depth:.2f}m, \nTarget: {self.approach_min_depth:.2f} m:{self.approach_max_depth:.2f}m")
            return None


    def get_average_marker_position(self):
        """
        Calculate the average position from recent marker history
        
        Returns:
            (avg_x, avg_y): Average coordinates, or None if insufficient data
        """
        if len(self.marker_positions) < self.min_positions_for_average:
            rospy.logwarn(f"Insufficient marker positions for averaging: {len(self.marker_positions)}/{self.min_positions_for_average}")
            if len(self.marker_positions) > 0:
                # Return the latest position if we have at least one
                return self.marker_positions[-1]
            return None
        
        # Calculate average
        avg_x = sum(pos[0] for pos in self.marker_positions) / len(self.marker_positions)
        avg_y = sum(pos[1] for pos in self.marker_positions) / len(self.marker_positions)
        
        # Calculate standard deviation to detect outliers
        if len(self.marker_positions) >= 5:
            std_dev_x = math.sqrt(sum((pos[0] - avg_x)**2 for pos in self.marker_positions) / len(self.marker_positions))
            std_dev_y = math.sqrt(sum((pos[1] - avg_y)**2 for pos in self.marker_positions) / len(self.marker_positions))
            rospy.logdebug(f"Position std dev: x={std_dev_x:.3f}m, y={std_dev_y:.3f}m")
        
        return avg_x, avg_y


            
    def _calculate_approach_waypoint(self, target_x, target_y, current_pose, retry=True):
        """
        Calculate the next waypoint for approaching the target
        
        Args:
            target_x: Target X coordinate
            target_y: Target Y coordinate
            current_pose: Current robot pose
            retry: Whether to retry with a smaller step if first attempt fails
            
        Returns:
            (next_x, next_y) or (None, None) if no safe path found
        """
        # Calculate next waypoint towards target
        next_x, next_y = calculate_safe_approach_point(
            target_x, target_y, current_pose, max_step=1.0
        )
        
        # Verify the calculated position is safe
        if not is_position_safe_approach(self.costmap, next_x, next_y):
            # If retry enabled, try a shorter step
            if retry:
                next_x, next_y = calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                if not is_position_safe_approach(self.costmap, next_x, next_y):
                    rospy.logerr("Cannot find safe approach path! line 239 object_approacher.py")
                    return None, None
            else:
                return None, None
                
        return next_x, next_y
            
    def _execute_approach_step(self, next_x, next_y, target_x, target_y, current_pose):
        """
        Execute a single step in the approach sequence
        
        Args:
            next_x: Next waypoint X coordinate
            next_y: Next waypoint Y coordinate
            target_x: Target X coordinate
            target_y: Target Y coordinate
            current_pose: Current robot pose
            
        Returns:
            None to continue approach
            True if target reached
            False if error occurred
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
        
        # Monitor the goal with a timeout
        timeout = rospy.Duration(self.navigation_timeout)
        rate = rospy.Rate(10)  # 10Hz checking rate
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Check if we've reached target depth during movement
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
                    rospy.logwarn(f"Unexpected state after cancellation: {self.nav_controller.client.get_state()}")
                    return False

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

if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        # You would need to create a node that instantiates both
        rospy.loginfo("ObjectApproacher cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object approacher interrupted")