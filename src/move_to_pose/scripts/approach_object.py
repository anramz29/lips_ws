#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion 
from nav_msgs.msg import OccupancyGrid
import sys
from tf.transformations import quaternion_from_euler
import math
import tf2_ros
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
import actionlib_msgs.msg
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# helper functions
from src.move_to_pose_utils import (
    load_poses,
    get_robot_pose,
    calculate_safe_approach_point,
    is_position_safe_approach
)

class ApproachObject:
    def __init__(self):
        rospy.init_node('approach_object_node', anonymous=True)

      
        # params
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.poses_config = rospy.get_param('~poses_config')
        self.pose_command = rospy.get_param('~pose_command', 'list')

        # Load poses from config
        self.poses = load_poses(self.poses_config)
        if not self.poses:
            rospy.logerr("Failed to load poses from config!")
            sys.exit(1)
        
        # Topic Params
        self.move_base_topic = rospy.get_param('~move_base_topic')
        self.costmap_topic = rospy.get_param('~costmap_topic')
        self.object_marker_topic = rospy.get_param('~object_marker_topic')

        # Create action client
        self.client = actionlib.SimpleActionClient(
            self.move_base_topic, 
            MoveBaseAction
        )

        rospy.loginfo("Waiting for locobot move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

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

        #instance variables for object maker
        self.object_marker = None
        self.current_scan_interrupted = False
        self.scanning_in_progress = True


        # Subscribe to object maker
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )
        rospy.loginfo("Received object maker")

        # Add subscriber for bbox depth
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', 'camera/yolo/bbox_depth')
        self.depth_sub = rospy.Subscriber(
            self.bbox_depth_topic,
            Float32MultiArray,  # Adjust this to match your actual message type
            self.depth_callback
        )
        self.current_depth = None
        self.object_reached = False

        self.cancel_pub = rospy.Publisher('/locobot/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=1)

    def object_marker_callback(self, msg):
        """Callback for receiving object marker messages"""
        if self.scanning_in_progress:
            if msg.header.frame_id != "map":
                rospy.logwarn(f"Marker not in map frame: {msg.header.frame_id}")
                return
            self.object_marker = msg
            self.current_scan_interrupted = True
        # if object already reached log info once and return
        elif self.object_reached:
            rospy.loginfo("Object already reached")
            return
        else:
            rospy.loginfo("Marker received but not scanning")

    def costmap_callback(self, msg):
        self.costmap = msg

    def depth_callback(self, msg):
        """Callback for receiving depth information"""
        if msg.data and len(msg.data) >= 5:
            self.current_depth = msg.data[4]
        else:
            rospy.logwarn("Invalid depth data received")

    def cancel_navigation(self):
        """Helper function to cancel navigation using both action client and topic"""
        # Cancel via action client
        self.client.cancel_all_goals()
        # Cancel via topic
        cancel_msg = actionlib_msgs.msg.GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Sent navigation cancellation commands")  

    def move_to_marker(self, marker):
        """Enhanced move to marker with graduated approach and robust cancellation"""
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        # Get initial robot pose
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        # Get target coordinates from marker
        target_x = marker.pose.position.x
        target_y = marker.pose.position.y
        
        while not rospy.is_shutdown():
            # First check if we have depth information
            if self.current_depth is None:
                rospy.logwarn("Waiting for depth information...")
                rospy.sleep(1)
                continue

            # Check if we've already reached the desired depth before doing any movement
            if 1.4 <= self.current_depth <= 1.6:
                rospy.loginfo(f"Already at target depth: {self.current_depth:.2f}m")
                # If there's an active goal, cancel it
                if self.client.get_state() == actionlib.GoalStatus.ACTIVE:
                    self.cancel_navigation()
                    rospy.sleep(0.5)  # Give time for cancellation to take effect
                return True

            # Calculate next waypoint towards target
            next_x, next_y = calculate_safe_approach_point(
                target_x, target_y, current_pose
            )
            
            # Verify the calculated position is safe
            if not is_position_safe_approach(self.costmap, next_x, next_y, current_pose):
                # Try a shorter step
                next_x, next_y = calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                if not is_position_safe_approach(next_x, next_y, current_pose):
                    rospy.logerr("Cannot find safe approach path!")
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
            timeout = rospy.Duration(10.0)
            rate = rospy.Rate(10)  # 10Hz checking rate
            start_time = rospy.Time.now()
            
            while not rospy.is_shutdown():
                # Check if we've reached target depth during movement
                if self.current_depth is not None and 1.4 <= self.current_depth <= 1.6:
                    rospy.loginfo(f"Reached target depth during movement: {self.current_depth:.2f}m")
                    self.cancel_navigation()
                    rospy.sleep(0.5)  # Wait for cancellation to take effect
                    # Verify cancellation state
                    if self.client.get_state() in [
                        actionlib.GoalStatus.PREEMPTED,
                        actionlib.GoalStatus.RECALLED,
                        actionlib.GoalStatus.ABORTED
                    ]:
                        return True
                    else:
                        rospy.logwarn(f"Unexpected state after cancellation: {self.client.get_state()}")
                        return False

                # Check if we've timed out
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logwarn("Goal timeout reached")
                    self.cancel_navigation()
                    return False

                # Check goal status
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    goal_reached = True
                    break
                elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                    rospy.logwarn("Goal aborted or rejected")
                    return False

                rate.sleep()

            # If we've reached the intermediate goal, update pose and continue
            if goal_reached:
                current_pose = get_robot_pose()
                if current_pose is None:
                    rospy.logerr("Failed to get updated robot pose")
                    return False
                continue  # Continue to next iteration of main loop

        return False  # If we get here, we've been shutdown

    def main(self):
        """Main loop for the approach object node"""

        rate = rospy.Rate(10)  # 10Hz
        rospy.sleep(5)  # wait 5 seconds before starting
        
        while not rospy.is_shutdown():
            if self.object_marker is not None and self.scanning_in_progress:
                rospy.loginfo("\n=== Starting Approach Sequence ===")
                
                # Get current robot position for logging/debugging
                current_pose = get_robot_pose()
                if current_pose is None:
                    rospy.logerr("Cannot get robot pose. Retrying...")
                    rate.sleep()
                    continue
                
                # Wait for depth information
                if self.current_depth is None:
                    rospy.logwarn("Waiting for depth information...")
                    rate.sleep()
                    continue
                    
                rospy.loginfo(f"Current depth to object: {self.current_depth:.2f}m")
                rospy.loginfo(f"Object position: x={self.object_marker.pose.position.x:.2f}, y={self.object_marker.pose.position.y:.2f}")
                rospy.loginfo(f"Robot position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
                
                if 1.4 <= self.current_depth <= 1.6:  # Check depth instead of geometric distance
                    rospy.loginfo(f"Already at correct viewing distance ({self.current_depth:.2f}m)")
                    self.scanning_in_progress = False
                    self.object_reached = True
                else:
                    success = self.move_to_marker(self.object_marker)
                    if success:
                        rospy.loginfo("Successfully completed approach sequence!")
                        self.scanning_in_progress = False
                    else:
                        rospy.logwarn("Failed to complete approach sequence. Will retry...")
                
                self.object_marker = None  # Reset marker after attempt
        
        rate.sleep()

if __name__ == '__main__':  
    try:
        node = ApproachObject()
        rospy.loginfo("=== Approach Object Node Started ===")
        rospy.loginfo("Waiting for object markers...")
        node.main()  # Run the main loop
    except rospy.ROSInterruptException:
        pass