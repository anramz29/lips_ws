#!/usr/bin/env python3
import rospy
import actionlib
import yaml
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
import sys
import math

class SearchAndApproachNode:
    def __init__(self):
        rospy.init_node('search_and_approach_node')
        
        # Initialize move_base client
        self.move_client = actionlib.SimpleActionClient('/locobot/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        
        # Load predefined poses
        self.poses = self.load_poses()
        if not self.poses:
            rospy.logerr("Failed to load poses!")
            sys.exit(1)
            
        # Subscribe to object marker
        self.object_sub = rospy.Subscriber(
            '/locobot/object_markers',
            Marker,
            self.object_callback
        )
        
        # Subscribe to depth information
        self.depth_sub = rospy.Subscriber(
            '/locobot/camera/yolo/bbox_depth',
            Float32MultiArray,
            self.depth_callback
        )
        
        # Initialize state variables
        self.object_pose = None
        self.current_depth = None
        self.target_distance = 1.5  # Desired distance from object in meters
        self.search_complete = False
        self.last_detection_time = None  # Add timestamp for detection
        self.detection_timeout = rospy.Duration(2.0)  # 2 second timeout
        
    def load_poses(self):
        """Load search poses from YAML config"""
        try:
            rospack = rospkg.RosPack()
            config_path = rospack.get_path('move_to_pose') + '/config/poses.yaml'
            with open(config_path, 'r') as file:
                return yaml.safe_load(file)['locations']
        except Exception as e:
            rospy.logerr(f"Error loading config: {str(e)}")
            return None
            
    def object_callback(self, msg):
        """Process detected object marker with timeout tracking"""
        self.object_pose = msg.pose
        self.last_detection_time = rospy.Time.now()
        if not self.search_complete and self.current_depth is not None:
            self.search_complete = True
            rospy.loginfo("Object detected! Starting approach...")
            
    def depth_callback(self, msg):
        """Process depth information with validation"""
        try:
            if len(msg.data) >= 5:
                depth = msg.data[4]
                if depth > 0:  # Validate depth value
                    self.current_depth = depth
                    rospy.logdebug(f"Updated depth: {self.current_depth}")
        except Exception as e:
            rospy.logwarn(f"Error processing depth: {e}")

    def is_detection_valid(self):
        """Check if we have valid and recent detection data"""
        if self.last_detection_time is None:
            return False
        
        # Check if detection is recent
        time_since_detection = rospy.Time.now() - self.last_detection_time
        if time_since_detection > self.detection_timeout:
            rospy.logwarn("Detection timeout - too old")
            return False
        
        # Check if we have both pose and depth
        if self.object_pose is None:
            rospy.logwarn("No object pose")
            return False
        
        if self.current_depth is None or self.current_depth <= 0:
            rospy.logwarn("No valid depth")
            return False
        
        return True
            
    def create_goal(self, pose):
        """Create MoveBaseGoal from pose"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal
        
    def move_to_pose(self, pose):
        """Move to a specific pose"""
        goal = self.create_goal(pose)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
        return self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED
    
    def approach_object(self):
        """Move towards object with continuous distance checking"""
        if not self.is_detection_valid():
            rospy.logwarn("Invalid detection state for approach")
            return False
            
        while not rospy.is_shutdown():
            if not self.is_detection_valid():
                rospy.logwarn("Lost valid detection during approach")
                return False
                
            # Calculate error in distance
            distance_error = self.current_depth - self.target_distance
            
            # If we're close enough, stop
            if abs(distance_error) < 0.1:
                rospy.loginfo(f"Reached target distance: {self.current_depth:.2f}m")
                return True
                
            # Calculate approach increment (smaller steps)
            # Move 20% of remaining distance each time
            increment = min(0.2 * distance_error, 0.3)  # Max 0.3m per move
            
            # Calculate approach pose
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = "map"
            approach_pose.header.stamp = rospy.Time.now()
            
            # Calculate direction vector
            dx = self.object_pose.position.x
            dy = self.object_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance <= 0:
                rospy.logwarn("Invalid distance calculation")
                return False
            
            # Move incrementally towards target
            scale = (distance - increment) / distance
            approach_pose.pose.position.x = dx * scale
            approach_pose.pose.position.y = dy * scale
            approach_pose.pose.position.z = 0
            
            # Calculate orientation towards object
            yaw = math.atan2(dy, dx)
            approach_pose.pose.orientation.z = math.sin(yaw/2)
            approach_pose.pose.orientation.w = math.cos(yaw/2)
            
            # Send the incremental move command
            goal = self.create_goal(approach_pose.pose)
            self.move_client.send_goal(goal)
            
            # Wait for smaller timeout for more frequent updates
            finished = self.move_client.wait_for_result(rospy.Duration(5.0))
            
            if not finished:
                rospy.logwarn("Move base timeout, recalculating...")
                self.move_client.cancel_goal()
                continue
                
            # Check if movement succeeded
            if self.move_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn("Move base failed to reach intermediate goal")
                return False
                
            # Brief pause to get new depth readings
            rospy.sleep(0.5)
            
            # Log progress
            rospy.loginfo(f"Current depth: {self.current_depth:.2f}m, Target: {self.target_distance:.2f}m")

    def search_and_approach(self):
        """Main execution loop with improved validation"""
        # Reset search state
        self.search_complete = False
        self.object_pose = None
        self.current_depth = None
        self.last_detection_time = None
        
        # Search through predefined poses
        for pose_name, pose_data in self.poses.items():
            if rospy.is_shutdown():
                return
                
            # Skip if object already found during movement
            if self.search_complete:
                rospy.loginfo("Object detected while moving! Starting approach...")
                break
                
            rospy.loginfo(f"Moving to search position: {pose_name}")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set position and orientation from pose data
            goal.target_pose.pose.position = Point(
                pose_data['position']['x'],
                pose_data['position']['y'],
                pose_data['position']['z']
            )
            goal.target_pose.pose.orientation = Quaternion(
                pose_data['orientation']['x'],
                pose_data['orientation']['y'],
                pose_data['orientation']['z'],
                pose_data['orientation']['w']
            )
            
            # Move to search position with periodic checks
            self.move_client.send_goal(goal)
            
            # Check for object while moving
            while not self.move_client.wait_for_result(rospy.Duration(0.5)):  # Check every 0.5 seconds
                if self.is_detection_valid() and self.search_complete:
                    rospy.loginfo("Object detected during movement!")
                    self.move_client.cancel_goal()  # Stop moving to the pose
                    break
            
            # If object not found during movement, search at current position
            if not self.search_complete:
                search_duration = rospy.Duration(5.0)
                search_start = rospy.Time.now()
                
                while (rospy.Time.now() - search_start) < search_duration:
                    if self.is_detection_valid() and self.search_complete:
                        rospy.loginfo(f"Object found at {pose_name}!")
                        break
                    rospy.sleep(0.1)
            
            if self.search_complete:
                break
        
        if not self.search_complete:
            rospy.logwarn("Object not found in any search position!")
            return
        
        # Start approach sequence with improved validation
        rospy.loginfo("Starting approach sequence...")
        max_approach_attempts = 3
        attempt = 0
        
        while not rospy.is_shutdown() and attempt < max_approach_attempts:
            if not self.is_detection_valid():
                rospy.logwarn("Lost valid detection, waiting for new data...")
                # Wait briefly for new detection
                rospy.sleep(0.5)
                if attempt > 0:  # Only increment attempts if we've already tried
                    attempt += 1
                continue
            
            if self.approach_object():
                rospy.loginfo("Successfully approached object!")
                break
            else:
                attempt += 1
                rospy.logwarn(f"Approach attempt {attempt} failed, retrying...")
                rospy.sleep(1.0)

def main():
    try:
        node = SearchAndApproachNode()
        node.search_and_approach()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()