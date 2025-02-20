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
from move_to_pose.src.move_to_pose_utils import load_poses, get_robot_pose

class LocobotMoveToPose:
    def __init__(self):
        rospy.init_node('find_object_node', anonymous=True)

      
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
        self.scanning_in_progress = False


        # Subscribe to object maker
        self.object_marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.object_marker_callback
        )
        rospy.loginfo("Received object maker")
   

    def costmap_callback(self, msg):
        self.costmap = msg

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
    

    def calculate_intermediate_goals(self, start_pose, final_goal):
        """Calculate intermediate goals every 2 meters"""
        goals = []
        
        # Calculate distance and direction
        dx = final_goal.position.x - start_pose.position.x
        dy = final_goal.position.y - start_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate number of intermediate points
        num_points = int(distance / 2.0)  # One point every 2 meters
        
        if num_points == 0:
            # Create a single MoveBaseGoal for the final position
            final_goal_msg = MoveBaseGoal()
            final_goal_msg.target_pose.header.frame_id = "map"
            final_goal_msg.target_pose.header.stamp = rospy.Time.now()
            final_goal_msg.target_pose.pose = final_goal
            return [final_goal_msg]
            
        # Create intermediate goals
        for i in range(num_points):
            fraction = (i + 1) / (num_points + 1)
            intermediate_goal = MoveBaseGoal()
            intermediate_goal.target_pose.header.frame_id = "map"
            intermediate_goal.target_pose.header.stamp = rospy.Time.now()
            
            # Interpolate position
            intermediate_goal.target_pose.pose.position.x = start_pose.position.x + dx * fraction
            intermediate_goal.target_pose.pose.position.y = start_pose.position.y + dy * fraction
            intermediate_goal.target_pose.pose.position.z = start_pose.position.z
            
            # Keep orientation pointing toward final goal
            angle = math.atan2(dy, dx)
            quaternion = quaternion_from_euler(0, 0, angle)
            intermediate_goal.target_pose.pose.orientation.x = quaternion[0]
            intermediate_goal.target_pose.pose.orientation.y = quaternion[1]
            intermediate_goal.target_pose.pose.orientation.z = quaternion[2]
            intermediate_goal.target_pose.pose.orientation.w = quaternion[3]
            
            goals.append(intermediate_goal)
        
        # Add final goal as MoveBaseGoal
        final_goal_msg = MoveBaseGoal()
        final_goal_msg.target_pose.header.frame_id = "map"
        final_goal_msg.target_pose.header.stamp = rospy.Time.now()
        final_goal_msg.target_pose.pose = final_goal
        goals.append(final_goal_msg)
        
        return goals

    def calculate_safe_approach_point(self, target_x, target_y, current_pose, max_step=1.0):
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
            current_pose = self.get_robot_pose()
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

    def rotate_to_angle(self, angle):
        """Rotate the robot to a specific angle"""
        scan_goal = MoveBaseGoal()
        scan_goal.target_pose.header.frame_id = "map"
        scan_goal.target_pose.header.stamp = rospy.Time.now()
        
        current_pose = self.get_robot_pose()
        if current_pose is None:
            rospy.logerr("Failed to get robot pose for scanning")
            return False
            
        scan_goal.target_pose.pose.position = current_pose.position
        
        quaternion = quaternion_from_euler(0, 0, angle)
        scan_goal.target_pose.pose.orientation = Quaternion(*quaternion)
        
        rospy.loginfo(f"Rotating to angle: {angle}")
        self.client.send_goal(scan_goal)
        return self.client.wait_for_result(rospy.Duration(5.0))

    def handle_marker_detection(self, saved_pose):
        """Handle the detection of a marker, including movement and return"""
        rospy.loginfo("Object detected! Waiting for marker position to stabilize...")

        # Store the time of detection
        detection_time = rospy.Time.now()
        while (rospy.Time.now() - detection_time) < rospy.Duration(2.0):
            rospy.sleep(0.1)

        # Move to the detected marker
        if self.move_to_marker(self.object_marker):
            rospy.sleep(2.0)  # Pause at marker position

            # Return to scan position
            rospy.loginfo("Returning to scan position...")
            return_goal = MoveBaseGoal()
            return_goal.target_pose.header.frame_id = "map"
            return_goal.target_pose.header.stamp = rospy.Time.now()
            return_goal.target_pose.pose = saved_pose

            self.client.send_goal(return_goal)
            self.client.wait_for_result(rospy.Duration(5.0))

        self.current_scan_interrupted = False
        self.object_marker = None
        return True

    def scan_at_position(self, duration=4.0):
        """Perform a scan at the current position for specified duration"""
        scan_start_time = rospy.Time.now()
        while (rospy.Time.now() - scan_start_time) < rospy.Duration(duration):
            if self.current_scan_interrupted and self.object_marker is not None:
                saved_pose = self.get_robot_pose()
                if self.handle_marker_detection(saved_pose):
                    return True  # Object found and handled
            rospy.sleep(0.1)
        return False  # No object found
    
    def perform_scan(self):
        """Perform a 360-degree scan by stopping at four directions"""
        # Initial scan at current position
        rospy.loginfo("Performing initial scan at current position...")
        self.scanning_in_progress = True
        self.current_scan_interrupted = False
        self.object_marker = None
        
        # Do initial scan
        if self.scan_at_position():
            return  # Object found during initial scan
        
        # If no object found, proceed with rotational scan
        angles = [0, math.pi/2, math.pi, -math.pi/2]
        
        for angle in angles:
            rospy.loginfo(f"Starting scan at {angle} radians")
            self.scanning_in_progress = True
            self.current_scan_interrupted = False
            self.object_marker = None
            
            if not self.rotate_to_angle(angle):
                continue
                
            if self.scan_at_position():
                return  # Object found during rotational scan
            
            self.scanning_in_progress = False
        
        rospy.loginfo("Completed full 360-degree scan")

    def move_to_named_pose(self, pose_name):
        """Move to a named pose with intermediate goals and scanning"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return False

        pose_data = self.poses[pose_name]
        final_goal = MoveBaseGoal()  # Changed from Pose() to MoveBaseGoal()
        final_goal.target_pose.header.frame_id = "map"
        final_goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set final goal position and orientation
        final_goal.target_pose.pose.position = Point(
            pose_data['position']['x'],
            pose_data['position']['y'],
            pose_data['position']['z']
        )
        final_goal.target_pose.pose.orientation = Quaternion(
            pose_data['orientation']['x'],
            pose_data['orientation']['y'],
            pose_data['orientation']['z'],
            pose_data['orientation']['w']
        )

        # Get current robot pose
        start_pose = self.get_robot_pose()
        if start_pose is None:
            rospy.logerr("Failed to get robot pose")
            return False

        # Calculate intermediate goals
        goals = self.calculate_intermediate_goals(start_pose, final_goal.target_pose.pose)
        
        # Execute each goal
        for i, goal in enumerate(goals):
            rospy.loginfo(f"Moving to {'intermediate' if i < len(goals)-1 else 'final'} goal {i+1}/{len(goals)}")
            
            # Check if position is safe
            if not self.is_position_safe(goal.target_pose.pose.position.x, 
                                    goal.target_pose.pose.position.y):
                rospy.logerr(f"Goal position is in unsafe area!")
                return False
            
            # Send goal and wait for result
            self.client.send_goal(goal)
            wait = self.client.wait_for_result()
            
            if not wait:
                rospy.logerr("Action server not available!")
                return False
            
            state = self.client.get_state()
            if state != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn(f"Goal failed with status: {str(state)}")
                return False
            
            # Perform scan at each intermediate point
            if i < len(goals)-1:  # Don't scan at final goal
                rospy.loginfo("Performing directional scan...")
                self.perform_scan()
        
        rospy.loginfo("Final goal reached successfully!")
        return True
    
    def list_available_poses(self):
        """List all available pose names"""
        rospy.loginfo("Available poses:")
        for pose_name in self.poses.keys():
            rospy.loginfo(f"- {pose_name}")

    def move_to_all_poses(self):
        """Move to all defined poses sequentially"""
        rospy.loginfo("Moving to all poses sequentially")
        for pose_name in self.poses.keys():
            rospy.loginfo(f"Moving to pose: {pose_name}")
            success = self.move_to_named_pose(pose_name)
            if not success:
                rospy.logerr(f"Failed to reach pose: {pose_name}")
                return False
            rospy.sleep(1.0)  # Small delay between poses
        return True
    
def main():
    try:
        mover = LocobotMoveToPose()
        
        command = mover.pose_command
        
        if command == 'list':
            mover.list_available_poses()
        elif command == 'all':
            mover.move_to_all_poses()
        else:
            success = mover.move_to_named_pose(command)
            
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")

if __name__ == '__main__':
    main()
