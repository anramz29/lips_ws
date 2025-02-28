#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
from object_scout.navigation_controller import NavigationController
from object_scout.object_scanner import ObjectScanner, ScanResult
from object_scout.object_approacher import ObjectApproacher
from object_scout.pose_manager import PoseManager
from object_scout.utils import get_robot_pose


class ScoutCoordinator:
    """
    Main coordinator for the object scouting mission
    """
    def __init__(self, init_node=True):
        """
        Initialize the coordinator
        
        Args:
            init_node: Whether to initialize a ROS node
        """
        if init_node:
            rospy.init_node('scout_coordinator', anonymous=False)
        
        # Get ROS parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '')
        self.pose_command = rospy.get_param('~pose_command', 'all')
        self.max_objects = rospy.get_param('~max_objects', 0)  # 0 means find all objects
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize components
        self.nav_controller = NavigationController(self.robot_name)
        self.pose_manager = PoseManager(self.poses_config)
        self.scanner = ObjectScanner(self.robot_name, self.nav_controller)
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller)
        
        # Track found objects
        self.found_objects = []

    def start_mission(self):
        """
        Start the object scouting mission
        """
        rospy.loginfo("Starting object scouting mission")
        poses = self.pose_manager.get_all_pose_names()

        for pose_name in poses:
            rospy.loginfo(f"Moving to pose: {pose_name}")
            pose_position = self.pose_manager.get_pose(pose_name)
            success = self.nav_controller.move_to_position(pose_position)
            
            if not success:
                rospy.logwarn(f"Failed to navigate to pose {pose_name}, continuing to next pose")
                continue

            # Check if we've already found enough objects
            if self.max_objects > 0 and len(self.found_objects) >= self.max_objects:
                rospy.loginfo(f"Reached maximum number of objects ({self.max_objects}), ending mission")
                break

            # Initial scan at the pose
            scan_result = self.scanner.perform_scan_rotation()
            
            # Loop to handle resuming scans and approaching objects
            while scan_result == ScanResult.OBJECT_DETECTED:
                # Get the object marker from the scanner
                object_marker = self.scanner.object_marker
                
                if object_marker is None:
                    rospy.logwarn("Object detected but no marker available")
                    break
                    
                # Approach the detected object
                approach_success = self.approacher.approach_object(object_marker)

                if approach_success:
                    rospy.loginfo(f"Successfully approached object at pose {pose_name}")
                    self.found_objects.append(pose_name)
                    
                    # Check if we've reached the maximum number of objects
                    if self.max_objects > 0 and len(self.found_objects) >= self.max_objects:
                        rospy.loginfo(f"Reached maximum number of objects ({self.max_objects}), ending mission")
                        break
                else:
                    rospy.logwarn("Failed to approach object")
                
                # Continue scanning from current position with an offset
                # regardless of whether approach was successful or not
                scan_result = self.scanner.resume_scan_with_offset()
                
                # If no more objects detected or an error occurred, exit the inner loop
                if scan_result != ScanResult.OBJECT_DETECTED:
                    rospy.loginfo("No more objects detected in current scan position")
                    break
                    
        rospy.loginfo(f"Mission complete. Found {len(self.found_objects)} objects: {self.found_objects}")
        return len(self.found_objects)

if __name__ == "__main__":
    try:
        coordinator = ScoutCoordinator(init_node=True)
        num_objects = coordinator.start_mission()
        if num_objects == 0:
            rospy.spin()  # Keep node running if no object was found
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator interrupted")