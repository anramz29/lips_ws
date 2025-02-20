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
from src.move_to_pose_utils import load_poses, get_robot_pose


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

    def object_marker_callback(self, msg):
        """Callback for receiving object marker messages"""
        if self.scanning_in_progress:
            if msg.header.frame_id != "map":
                rospy.logwarn(f"Marker not in map frame: {msg.header.frame_id}")
                return
            self.object_marker = msg
            self.current_scan_interrupted = True
        else:
            rospy.loginfo("Marker received but not scanning")

    def costmap_callback(self, msg):
        self.costmap = msg
        
    
    def calculate_safe_approach_point(self, target_x, target_y, current_pose, max_step=2.0):
        """Calculate a safe intermediate point towards the target"""
        dx = target_x - current_pose.position.x
        dy = target_y - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= max_step:
            return target_x, target_y
            
        # Normalize direction vector and scale by max_step
        dx = (dx/distance) * max_step
        dy = (dy/distance) * max_step
        
        return current_pose.position.x + dx, current_pose.position.y + dy

    def get_adaptive_cost_threshold(self, distance):
        """Get cost threshold that becomes more lenient with distance"""
        # Base threshold for close distances
        base_threshold = 90
        # Reduce threshold (become more lenient) as distance increases
        distance_factor = min(distance / 2.0, 1.0)  # Cap at 2 meters
        return base_threshold * (1.0 - distance_factor * 0.3)  # Can go down to 70% of base threshold

    def is_position_safe(self, x, y, current_pose=None):
        """Enhanced safety check with distance-based threshold"""
        if self.costmap is None:
            rospy.logwarn("No costmap available")
            return False

        # Convert world coordinates to costmap cell coordinates
        cell_x = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        cell_y = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)

        # Check if coordinates are within costmap bounds
        if (cell_x < 0 or cell_x >= self.costmap.info.width or
            cell_y < 0 or cell_y >= self.costmap.info.height):
            rospy.logwarn(f"Position ({x}, {y}) is outside costmap bounds")
            return False

        # Get cost value at position
        index = cell_y * self.costmap.info.width + cell_x
        cost = self.costmap.data[index]

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

    def move_to_marker(self, marker):
        """Enhanced move to marker with graduated approach"""
        rospy.loginfo("Moving to detected object with graduated approach...")
        
        current_pose = get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False
        
        target_x = marker.pose.position.x
        target_y = marker.pose.position.y
        
        while True:
            # Calculate next safe approach point
            next_x, next_y = self.calculate_safe_approach_point(
                target_x, target_y, current_pose
            )
            
            # Check if position is safe with current pose context
            if not self.is_position_safe(next_x, next_y, current_pose):
                # Try a shorter step
                next_x, next_y = self.calculate_safe_approach_point(
                    target_x, target_y, current_pose, max_step=0.5
                )
                if not self.is_position_safe(next_x, next_y, current_pose):
                    rospy.logerr("Cannot find safe approach path!")
                    return False
            
            # Create and send goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            goal.target_pose.pose.position.x = next_x
            goal.target_pose.pose.position.y = next_y
            goal.target_pose.pose.position.z = current_pose.position.z
            
            # Calculate orientation to face the final target
            dx = target_x - next_x
            dy = target_y - next_y
            angle = math.atan2(dy, dx)
            quaternion = quaternion_from_euler(0, 0, angle)
            goal.target_pose.pose.orientation = Quaternion(*quaternion)
            
            # Send goal and wait for result
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(10.0))
            
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                rospy.logerr("Failed to reach intermediate position")
                return False
            
            # Update current pose
            current_pose = get_robot_pose()
            if current_pose is None:
                rospy.logerr("Failed to get updated robot pose")
                return False
            
            # Check if we've reached the target (within tolerance)
            current_distance = math.sqrt(
                (target_x - current_pose.position.x)**2 + 
                (target_y - current_pose.position.y)**2
            )
            
            if current_distance < 0.1:  # 10cm tolerance
                rospy.loginfo("Successfully reached marker position")
                return True
    

    def main(self):
        """Main execution loop for approaching objects"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.object_marker is not None and self.scanning_in_progress:
                rospy.loginfo("\n=== Starting Approach Sequence ===")
                
                # Get current robot position
                current_pose = get_robot_pose()
                if current_pose is None:
                    rospy.logerr("Cannot get robot pose. Retrying...")
                    rate.sleep()
                    continue
                
                # Calculate distance to object
                target_x = self.object_marker.pose.position.x
                target_y = self.object_marker.pose.position.y
                distance_to_object = math.sqrt(
                    (target_x - current_pose.position.x)**2 + 
                    (target_y - current_pose.position.y)**2
                )
                
                rospy.loginfo(f"Distance to object: {distance_to_object:.2f} meters")
                rospy.loginfo(f"Object position: x={target_x:.2f}, y={target_y:.2f}")
                rospy.loginfo(f"Robot position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
                
                if distance_to_object < 0.1:  # Already at target
                    rospy.loginfo("Already at target position!")
                else:
                    # Attempt to move to the marker
                    success = self.move_to_marker(self.object_marker)
                    if success:
                        rospy.loginfo("Successfully completed approach sequence!")
                        self.scanning_in_progress = False  # Stop scanning after successful approach
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