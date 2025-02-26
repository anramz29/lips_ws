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

            # Set up object marker subscription
        self.object_marker_topic = rospy.get_param('~object_marker_topic', 
                                                f'/{robot_name}/object_markers')
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,  # Assuming this is the correct message type
            self.object_marker_callback
        )

        self.object_marker = None    # Set up object marker subscription


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
        Approach a detected object with improved marker persistence
        
        Args:
            object_marker: The marker for the detected object
            
        Returns:
            bool: Success flag
        """
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        if object_marker is None:
            rospy.logerr("No object marker available to approach")
            return False
        
        # Store initial marker details as a fallback
        initial_target_x = object_marker.pose.position.x
        initial_target_y = object_marker.pose.position.y
        
        # Get initial robot pose (save for potential fallback)
        original_pose = get_robot_pose()
        if original_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        # Track detection reliability
        consecutive_detection_failures = 0
        max_detection_failures = 5
        
        # Approach loop
        while not rospy.is_shutdown():
            # If we've lost too many detections, abort
            if consecutive_detection_failures >= max_detection_failures:
                rospy.logwarn("Repeated detection failures. Aborting approach.")
                return False
            
            # Use initial target if marker is lost
            target_x = (self.object_marker.pose.position.x 
                        if self.object_marker is not None 
                        else initial_target_x)
            target_y = (self.object_marker.pose.position.y 
                        if self.object_marker is not None 
                        else initial_target_y)
            
            # Depth and marker validation
            depth_check = self._depth_and_marker_check(
                initial_target_x, initial_target_y
            )
            
            if depth_check is True:
                return True
            elif depth_check is False:
                consecutive_detection_failures += 1
                rospy.logwarn(f"Detection failure. Attempts: {consecutive_detection_failures}")
                rospy.sleep(0.5)  # Brief pause between attempts
                continue
            
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
                return True
        
        return False

    def _depth_and_marker_check(self, initial_x, initial_y):
        """
        Enhanced depth and marker validation
        
        Args:
            initial_x: Initial target X coordinate
            initial_y: Initial target Y coordinate
        
        Returns:
            None: Continue approach
            True: Successfully reached target
            False: Detection failure
        """
        # Comprehensive logging
        rospy.logdebug(f"Depth check - Current depth: {self.current_depth}")
        rospy.logdebug(f"Object marker status: {'Present' if self.object_marker is not None else 'None'}")
        
        # Depth information check
        if self.current_depth is None:
            rospy.logwarn("Waiting for depth information...")
            rospy.sleep(0.5)
            return None
        
        # Depth target check
        if self.approach_min_depth <= self.current_depth <= self.approach_max_depth:
            rospy.loginfo(f"Reached target depth: {self.current_depth:.2f}m")
            
            # Cancel active navigation if needed
            if self.nav_controller.client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.nav_controller.cancel_navigation()
                rospy.sleep(0.5)
            
            return True
        
        # If no marker, use initial coordinates for logging
        marker_x = (self.object_marker.pose.position.x 
                    if self.object_marker is not None 
                    else initial_x)
        marker_y = (self.object_marker.pose.position.y 
                    if self.object_marker is not None 
                    else initial_y)
        
        # Marker validation with more context
        if self.object_marker is None:
            rospy.logwarn(f"Lost object marker. Last known position: x={initial_x}, y={initial_y}")
            rospy.logwarn(f"Current depth: {self.current_depth}")
            return False
        
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