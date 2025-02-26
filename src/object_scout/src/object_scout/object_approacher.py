#!/usr/bin/env python3
import rospy
import math
import actionlib
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

# Import utilities
from object_scout.utils import get_robot_pose, calculate_safe_approach_point, is_position_safe_approach


class ObjectApproacher:
    """
    Handles approaching detected objects
    """
    def __init__(self, robot_name, nav_controller, init_node=False):
        """
        Initialize the approacher
        
        Args:
            robot_name: Name of the robot for topic namespacing
            nav_controller: Instance of NavigationController for movement commands
            init_node: Whether to initialize a ROS node (standalone mode)
        """
        if init_node:
            rospy.init_node('object_approacher', anonymous=False)
            
        self.robot_name = robot_name
        self.nav_controller = nav_controller
        self.costmap = nav_controller.costmap
        
        # Approacher state
        self.current_depth = None
        self.MAX_DETECTION_FAILURES = 3
        
        # Approach parameters
        self.approach_min_depth = 1.0  # meters
        self.approach_max_depth = 1.2  # meters
        self.navigation_timeout = 40.0  # seconds
        
        # Set up depth subscription
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 
                                               f'/{robot_name}/camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )

        self.object_marker = None

    def object_marker_callback(self, msg):
        """
        Process object marker messages
        
        Args:
            msg: Marker message from object detection
        """
        # Store the latest marker for approach
        self.object_marker = msg

    def approach_object(self, object_marker):
        """
        Approach a detected object with improved robustness and fallback mechanisms
        
        Args:
            object_marker: The marker for the detected object
            
        Returns:
            bool: Success flag
        """
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        if object_marker is None:
            rospy.logerr("No object marker available to approach")
            return False
            
        # Get initial robot pose (save for potential fallback)
        original_pose = get_robot_pose()
        if original_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        # Get target coordinates from marker
        target_x = object_marker.pose.position.x
        target_y = object_marker.pose.position.y
        
        # IMPORTANT: Initialize current_pose here
        current_pose = original_pose
        
        # Track detection reliability
        consecutive_detection_failures = 0
        
        # Approach loop
        while not rospy.is_shutdown():
            # Check depth information and detection reliability
            approach_result = self._check_approach_depth_and_reliability()
            if approach_result is not None:
                return approach_result
            
            # If we've lost detection multiple times, consider fallback strategy
            if consecutive_detection_failures >= self.MAX_DETECTION_FAILURES:
                rospy.logwarn("Repeated detection failures. Initiating fallback strategy.")
                
                # Option 1: Return to original pose
                rospy.loginfo("Returning to original pose")
                fallback_success = self.nav_controller.move_to_position(
                    original_pose.position.x,
                    original_pose.position.y
                )
                
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
            
            # Track detection reliability
            if approach_result is False:
                consecutive_detection_failures += 1
            elif approach_result is True:
                return True
            
            # Update current pose for next iteration
            current_pose = get_robot_pose()
            if current_pose is None:
                rospy.logerr("Failed to get updated robot pose")
                return False
        
        return False 
    

    def _check_approach_depth_and_reliability(self):
        """
        Enhanced depth and detection reliability check
        
        Returns:
            None if approach should continue
            True if target depth reached
            False if error occurred
        """
        # First check if we have depth information
        if self.current_depth is None:
            rospy.logwarn("Waiting for depth information...")
            rospy.sleep(1)
            return None

        # Check if object marker is still valid
        if self.object_marker is None:
            rospy.logwarn("Lost object marker during approach")
            return False

        # Check depth criteria
        if self.approach_min_depth <= self.current_depth <= self.approach_max_depth:
            rospy.loginfo(f"Reached target depth: {self.current_depth:.2f}m")
            # If there's an active goal, cancel it
            if self.nav_controller.client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.nav_controller.cancel_navigation()
                rospy.sleep(0.5)  # Give time for cancellation to take effect
            return True
        
        return None

            
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
            target_x, target_y, current_pose
        )
        
        # Verify the calculated position is safe
        if not is_position_safe_approach(self.costmap, next_x, next_y):
            # If retry enabled, try a shorter step
            if retry:
                next_x, next_y = calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                if not is_position_safe_approach(self.costmap, next_x, next_y):
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
        
    def depth_callback(self, msg):
        """
        Process depth information of detected objects
        
        Args:
            msg: Float32MultiArray containing depth data
        """
        if msg.data and len(msg.data) >= 5:
            self.current_depth = msg.data[4]
        else:
            rospy.logwarn("Invalid depth data received")


if __name__ == "__main__":
    try:
        # This class requires a NavigationController, so we can't run it directly
        # You would need to create a node that instantiates both
        rospy.loginfo("ObjectApproacher cannot be run directly as it requires a NavigationController")
    except rospy.ROSInterruptException:
        rospy.loginfo("Object approacher interrupted")