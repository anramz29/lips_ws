#!/usr/bin/env python3
import rospy
import actionlib
import yaml
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from visualization_msgs.msg import Marker
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.move_to_pose_utils import (
    load_poses,
    get_robot_pose,
    is_position_safe,
    calculate_intermediate_point,
    calculate_safe_approach_point,
    is_position_safe_approach
)
from std_msgs.msg import Float32MultiArray
from actionlib_msgs.msg import GoalID
import actionlib_msgs.msg


class MoveAndScanAndApproach:
    def __init__(self):
        rospy.init_node('scan_and_approach_node', anonymous=False)
        
        # Get ROS parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '/home/rosuser/lips_ws/src/move_to_pose/config/poses.yaml')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        
        # Get topic parameters with default values that use robot_name
        self.move_base_topic = rospy.get_param('~move_base_topic', f'/{self.robot_name}/move_base')
        self.costmap_topic = rospy.get_param('~costmap_topic', f'/{self.robot_name}/move_base/global_costmap/costmap')
        
        # Object detection parameters
        self.object_marker_topic = rospy.get_param('~object_marker_topic', f'/{self.robot_name}/detected_object/marker')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', f'/{self.robot_name}/camera/yolo/bbox_depth')
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Create action client using the configured move_base topic
        self.client = actionlib.SimpleActionClient(self.move_base_topic, MoveBaseAction)
        rospy.loginfo(f"Waiting for {self.robot_name} move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Subscribe to costmap using the configured topic
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

        # Load poses from config
        self.poses = load_poses(self.poses_config)
        
        if not self.poses:
            rospy.logerr(f"Failed to load poses from config file: {self.poses_config}")
            sys.exit(1)
            
        rospy.loginfo(f"Loaded {len(self.poses)} poses from {self.poses_config}")
        rospy.loginfo(f"Pose command: {self.pose_command}")

        # Setup for object detection
        self.object_marker = None
        self.object_detected = False
        self.scanning_in_progress = False
        self.current_depth = None
        self.sustained_detection_start = None
        self.required_detection_duration = rospy.Duration(2.0)  # 2 seconds sustained detection
        
        # Subscribe to object marker
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )
        
        # Subscribe to bbox depth
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,
            self.depth_callback
        )
        
        # Setup cancel publisher
        self.cancel_pub = rospy.Publisher(f'{self.move_base_topic}/cancel', actionlib_msgs.msg.GoalID, queue_size=1)

    # Callback functions
    def costmap_callback(self, msg):
        """Callback for costmap updates"""
        self.costmap = msg

    def object_marker_callback(self, msg):
        """Callback for receiving object marker messages with sustained detection"""
        if self.scanning_in_progress:
            if msg.header.frame_id != "map":
                rospy.logwarn(f"Marker not in map frame: {msg.header.frame_id}")
                return
    
            # Store the validated marker
            self.object_marker = msg
    
    def reset_detection_state(self):
        """Reset detection state between scans"""
        self.object_detected = False
        self.object_marker = None
        self.sustained_detection_start = None

    def depth_callback(self, msg):
        """Callback for receiving depth information"""
        if msg.data and len(msg.data) >= 5:
            self.current_depth = msg.data[4]
        else:
            rospy.logwarn("Invalid depth data received")

    # Navigation helper functions
    def cancel_navigation(self):
        """Helper function to cancel navigation using both action client and topic"""
        # Cancel via action client
        self.client.cancel_all_goals()
        # Cancel via topic
        cancel_msg = actionlib_msgs.msg.GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Sent navigation cancellation commands")

    def perform_scan_rotation(self):
        """Perform rotations to cover a full 360-degree view with stabilization and sustained detection"""
        rospy.loginfo("Starting scan rotation sequence with five rotations")
        
        # Set scanning flag to true
        self.scanning_in_progress = True
        self.object_detected = False
        
        # Get current position
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            self.scanning_in_progress = False
            return False
        
        # Create a goal that maintains current position but changes orientation
        current_goal = MoveBaseGoal()
        current_goal.target_pose.header.frame_id = "map"
        current_goal.target_pose.pose.position = Point(
            current_pose.position.x,
            current_pose.position.y,
            0
        )
        
        # Five rotations of 72 degrees each to cover 360 degrees
        rotation_angles = [0, 60, 120, 180, 240, 300]  # Degrees
        num_rotations = len(rotation_angles)
        
        # Perform the rotations
        for i, angle in enumerate(rotation_angles):
            # Reset object detection status for this rotation
            self.object_marker = None
            self.object_detected = False
            
            # Calculate orientation in radians
            yaw = math.radians(angle)
            quaternion = quaternion_from_euler(0, 0, yaw)
            
            # Update goal orientation
            current_goal.target_pose.header.stamp = rospy.Time.now()
            current_goal.target_pose.pose.orientation = Quaternion(*quaternion)
            
            # Send rotation goal
            rospy.loginfo(f"Executing rotation {i+1}/{num_rotations} ({angle} degrees)")
            self.client.send_goal(current_goal)
            
            # Wait for rotation to complete
            timeout = rospy.Duration(45.0)  # 30 seconds timeout
            
            if not self.client.wait_for_result(timeout):
                rospy.logwarn("Rotation timeout reached")
                continue
            
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn("Rotation goal failed")
                continue
            
            # Rotation complete, wait for camera to stabilize to avoid motion blur
            rospy.loginfo("Rotation complete, waiting for camera stabilization...")
            stabilization_time = 1.0  # 1 second to let camera stabilize
            rospy.sleep(stabilization_time)
            
            # Clear any previous detection that might be from motion blur
            self.object_marker = None
            
            # Now scan for objects with sustained detection requirement
            rospy.loginfo(f"Starting object detection at {angle} degrees...")
            
            # Variables for sustained detection
            detection_start = None
            required_duration = rospy.Duration(2.0)  # Require 1 second of sustained detection
            scan_timeout = rospy.Duration(3.0)  # Total scan time at this position
            scan_start = rospy.Time.now()
            
            while (rospy.Time.now() - scan_start) < scan_timeout and not rospy.is_shutdown():
                # Check if we have a marker
                if self.object_marker is not None:
                    # Start or continue timing the sustained detection
                    if detection_start is None:
                        detection_start = rospy.Time.now()
                        rospy.loginfo("Potential object detected, timing sustained detection...")
                    else:
                        # Check if we've maintained detection long enough
                        elapsed = rospy.Time.now() - detection_start
                        if elapsed >= required_duration:
                            self.object_detected = True
                            rospy.loginfo(f"Sustained object detection for {elapsed.to_sec():.2f} seconds!")
                            break
                else:
                    # Lost detection, reset the timer
                    if detection_start is not None:
                        rospy.logwarn("Lost detection during sustainment period, resetting timer")
                        detection_start = None
                
                rospy.sleep(0.1)  # Check at 10Hz
            
            # If object detected with sustained requirement, exit the rotation sequence
            if self.object_detected:
                rospy.loginfo("Confirmed object detection, stopping scan sequence")
                break
                    
            # Brief pause between rotations
            if i < len(rotation_angles) - 1:  # Don't pause after the last rotation
                rospy.sleep(0.5)
        
        # End of rotation sequence
        detected = self.object_detected
        
        # Reset scanning flag if no detection
        if not detected:
            self.scanning_in_progress = False
            rospy.loginfo("Completed scan rotation sequence, no objects detected")
        
        return detected

    def move_to_position(self, x, y, orientation=None, scan_at_goal=False, timeout=30.0):
        """Move to a specific position with optional orientation"""
        current_pose = get_robot_pose()

        if current_pose is not None:
            if not is_position_safe(self.costmap, x, y):
                rospy.logerr(f"Position ({x}, {y}) is in unsafe area!")
                return False
        else:
            rospy.logerr("Failed to get current robot pose")
            return False

        # Create and send goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        
        # Use provided orientation or default
        if orientation:
            goal.target_pose.pose.orientation = orientation
        else:
            quaternion = quaternion_from_euler(0, 0, 0)
            goal.target_pose.pose.orientation = Quaternion(*quaternion)

        rospy.loginfo(f"Moving to position: x={x:.2f}, y={y:.2f}")
        self.client.send_goal(goal)
        
        # Wait for result with timeout
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
        
        # Perform a scan if requested and movement was successful
        if success and scan_at_goal:
            rospy.loginfo("Performing scan at goal position")
            detection = self.perform_scan_rotation()
            
            # If object detected during scan, approach it
            if detection:
                return self.approach_detected_object()
            
        return success

    def move_to_named_pose(self, pose_name):
        """Move to a named pose with a single intermediate scanning point"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return False
        
        # reset object detection flag
        self.object_detected = False

        # Get current robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current robot pose")
            return False

        # Get target pose
        target_pose = Pose()
        target_pose.position.x = self.poses[pose_name]['position']['x']
        target_pose.position.y = self.poses[pose_name]['position']['y']
        
        # Calculate halfway point
        int_x, int_y = calculate_intermediate_point(current_pose, target_pose)
        
        # Move to intermediate point
        rospy.loginfo(f"Moving to intermediate point: x={int_x:.2f}, y={int_y:.2f}")
        if not self.move_to_position(int_x, int_y):
            rospy.logerr("Failed to reach intermediate point")
            return False
            
        # Perform scan at intermediate point
        rospy.loginfo("Scanning at intermediate point...")
        detection = self.perform_scan_rotation()
        
        # If object detected during scan, approach it
        if detection:
            result = self.approach_detected_object()
            rospy.loginfo(f"Object approach at intermediate point result: {result}")
            return True  # We consider the mission a success if we approached an object
        
        # Move to final destination and scan there
        rospy.loginfo(f"Moving to final position: {pose_name}")
        result = self.move_to_position(
            target_pose.position.x,
            target_pose.position.y,
            scan_at_goal=True  # Perform scan at the final destination
        )
        
        return result

    def approach_detected_object(self):
        """Approach a detected object with graduated approach"""
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        if self.object_marker is None:
            rospy.logerr("No object marker available to approach")
            self.scanning_in_progress = False
            return False
            
        # Get initial robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get robot pose")
            self.scanning_in_progress = False
            return False
        
        # Get target coordinates from marker
        target_x = self.object_marker.pose.position.x
        target_y = self.object_marker.pose.position.y
        
        while not rospy.is_shutdown():
            # First check if we have depth information
            if self.current_depth is None:
                rospy.logwarn("Waiting for depth information...")
                rospy.sleep(1)
                continue

            # Check if we've already reached the desired depth before doing any movement
            if 1.0 <= self.current_depth <= 1.2:
                rospy.loginfo(f"Already at target depth: {self.current_depth:.2f}m")
                # If there's an active goal, cancel it
                if self.client.get_state() == actionlib.GoalStatus.ACTIVE:
                    self.cancel_navigation()
                    rospy.sleep(0.5)  # Give time for cancellation to take effect
                self.scanning_in_progress = False
                return True

            # Calculate next waypoint towards target
            next_x, next_y = calculate_safe_approach_point(
                target_x, target_y, current_pose
            )
            
            # Verify the calculated position is safe
            if not is_position_safe_approach(self.costmap, next_x, next_y):
                # Try a shorter step
                next_x, next_y = calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                if not is_position_safe_approach(self.costmap, next_x, next_y):
                    rospy.logerr("Cannot find safe approach path!")
                    self.scanning_in_progress = False
                    return False
            
            # Create the navigation goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set position
            goal.target_pose.pose.position.x = next_x
            goal.target_pose.pose.position.y = next_y
            goal.target_pose.pose.position.z = current_pose.position.z
            
            # Calculate orientation to face the target
            dx = target_x - next_x
            dy = target_y - next_y
            angle = math.atan2(dy, dx)
            quaternion = quaternion_from_euler(0, 0, angle)
            goal.target_pose.pose.orientation = Quaternion(*quaternion)
            
            # Send the goal
            self.client.send_goal(goal)
            
            # Monitor the goal with a timeout
            goal_reached = False
            timeout = rospy.Duration(40.0)
            rate = rospy.Rate(10)  # 10Hz checking rate
            start_time = rospy.Time.now()
            
            while not rospy.is_shutdown():
                # Check if we've reached target depth during movement
                if self.current_depth is not None and 1.0 <= self.current_depth <= 1.2:
                    rospy.loginfo(f"Reached target depth during movement: {self.current_depth:.2f}m")
                    self.cancel_navigation()
                    rospy.sleep(0.5)  # Wait for cancellation to take effect
                    # Verify cancellation state
                    if self.client.get_state() in [
                        actionlib.GoalStatus.PREEMPTED,
                        actionlib.GoalStatus.RECALLED,
                        actionlib.GoalStatus.ABORTED
                    ]:
                        self.scanning_in_progress = False
                        return True
                    else:
                        rospy.logwarn(f"Unexpected state after cancellation: {self.client.get_state()}")
                        self.scanning_in_progress = False
                        return False

                # Check if we've timed out
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logwarn("Goal timeout reached")
                    self.cancel_navigation()
                    self.scanning_in_progress = False
                    return False

                # Check goal status
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    goal_reached = True
                    break
                elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                    rospy.logwarn("Goal aborted or rejected")
                    self.scanning_in_progress = False
                    return False

                rate.sleep()

            # If we've reached the intermediate goal, update pose and continue
            if goal_reached:
                current_pose = get_robot_pose()
                if current_pose is None:
                    rospy.logerr("Failed to get updated robot pose")
                    self.scanning_in_progress = False
                    return False
                continue  # Continue to next iteration of main loop

        self.scanning_in_progress = False
        return False  # If we get here, we've been shutdown

    def move_to_all_poses(self):
        """Move to each pose in sequence"""
        found_object = False
        
        for pose_name in self.poses.keys():
            if found_object:
                rospy.loginfo("Object already found and approached, ending navigation sequence")
                break
                
            rospy.loginfo(f"\nMoving to: {pose_name}")
            success = self.move_to_named_pose(pose_name)
            
            
            # if success and object detected, set flag
            if success and self.object_detected:
                found_object = True
                rospy.loginfo(f"Object detected at {pose_name}, stopping navigation sequence")

            if not success:
                rospy.logwarn(f"Failed to reach {pose_name}, continuing to next pose...")
                
            rospy.sleep(1)  # Brief pause between movements
            
        return found_object

    def main(self):
        """Main execution function"""
        rospy.loginfo("=== Move, Scan and Approach Node Started ===")
        result = self.move_to_all_poses()
        
        if result:
            rospy.loginfo("Successfully completed navigation and found an object!")
        else:
            rospy.loginfo("Completed navigation sequence, no objects approached")
            
        return result


if __name__ == '__main__':
    try:
        node = MoveAndScanAndApproach()
        result = node.main()
        if not result:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")