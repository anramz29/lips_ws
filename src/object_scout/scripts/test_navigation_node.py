#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg
import time
import threading
from geometry_msgs.msg import Quaternion

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

from object_scout.navigation_controller import NavigationController
from object_scout.pose_manager import PoseManager
from object_scout.utils import get_robot_pose

class TestNavigationNode:
    """
    Test node for NavigationController and PoseManager
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('test_navigation')
        
        # Get parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.poses_config = rospy.get_param('~poses_config', '')
        
        # Initialize the components for testing
        self.nav_controller = NavigationController(self.robot_name, init_node=False)
        self.pose_manager = PoseManager(self.poses_config, init_node=False)
        
        # Create a simple menu for interaction
        self.menu_thread = threading.Thread(target=self.menu_loop)
        self.menu_thread.daemon = True
        self.menu_thread.start()
        
        rospy.loginfo("=== Navigation and Pose Manager Test Node Started ===")
        rospy.loginfo("Use the console menu to test navigation and pose management")
        
    def menu_loop(self):
        """
        Interactive menu loop for testing
        """
        time.sleep(1)  # Brief pause to let ROS initialize
        
        while not rospy.is_shutdown():
            print("\n=== Navigation Test Menu ===")
            print("1. List all available poses")
            print("2. Navigate to named pose")
            print("3. Navigate to custom position")
            print("4. Get current robot pose")
            print("5. Cancel current navigation")
            print("6. Test sequential navigation to all poses")
            print("0. Exit")
            
            try:
                choice = input("Enter your choice: ").strip()
                
                if choice == '1':
                    self.list_poses()
                elif choice == '2':
                    self.navigate_to_named_pose()
                elif choice == '3':
                    self.navigate_to_custom_position()
                elif choice == '4':
                    self.show_current_pose()
                elif choice == '5':
                    self.cancel_navigation()
                elif choice == '6':
                    self.test_sequential_navigation()
                elif choice == '0':
                    rospy.signal_shutdown("User requested exit")
                    break
                else:
                    print("Invalid choice. Please try again.")
            except Exception as e:
                rospy.logerr(f"Error in menu: {str(e)}")
            
            time.sleep(0.5)
    
    def list_poses(self):
        """List all available poses"""
        pose_names = self.pose_manager.get_all_pose_names()
        print("\nAvailable poses:")
        
        for name in pose_names:
            pose = self.pose_manager.get_pose(name)
            print(f"  - {name}: x={pose.position.x:.2f}, y={pose.position.y:.2f}")
    
    def navigate_to_named_pose(self):
        """Navigate to a named pose"""
        self.list_poses()
        pose_name = input("\nEnter pose name: ").strip()
        
        if not pose_name:
            print("No pose name provided. Aborting.")
            return
            
        pose = self.pose_manager.get_pose(pose_name)
        if pose is None:
            print(f"Invalid pose name: {pose_name}")
            return
            
        print(f"Navigating to {pose_name}: x={pose.position.x:.2f}, y={pose.position.y:.2f}")
        
        # Send the navigation command
        result = self.nav_controller.move_to_position(
            pose.position.x,
            pose.position.y,
            pose.orientation if hasattr(pose, 'orientation') else None
        )
        
        if result:
            print(f"Successfully navigated to {pose_name}")
        else:
            print(f"Failed to navigate to {pose_name}")
    
    def navigate_to_custom_position(self):
        """Navigate to a custom position"""
        try:
            x = float(input("Enter X coordinate: ").strip())
            y = float(input("Enter Y coordinate: ").strip())
            
            print(f"Navigating to custom position: x={x:.2f}, y={y:.2f}")
            
            # Send the navigation command
            result = self.nav_controller.move_to_position(x, y)
            
            if result:
                print("Successfully navigated to custom position")
            else:
                print("Failed to navigate to custom position")
                
        except ValueError:
            print("Invalid coordinates. Please enter valid numbers.")
    
    def show_current_pose(self):
        """Show the current robot pose"""
        current_pose = get_robot_pose()
        
        if current_pose is not None:
            print("\nCurrent pose:")
            print(f"  Position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}, z={current_pose.position.z:.2f}")
            print(f"  Orientation: x={current_pose.orientation.x:.2f}, y={current_pose.orientation.y:.2f}, z={current_pose.orientation.z:.2f}, w={current_pose.orientation.w:.2f}")
        else:
            print("Failed to get current robot pose")
    
    def cancel_navigation(self):
        """Cancel the current navigation goal"""
        self.nav_controller.cancel_navigation()
        print("Navigation cancelled")
    
    def test_sequential_navigation(self):
        """Test navigation to all poses in sequence"""
        pose_names = self.pose_manager.get_all_pose_names()
        
        print(f"\nSequentially navigating to {len(pose_names)} poses...")
        
        for i, pose_name in enumerate(pose_names):
            pose = self.pose_manager.get_pose(pose_name)
            
            print(f"\n[{i+1}/{len(pose_names)}] Navigating to {pose_name}: x={pose.position.x:.2f}, y={pose.position.y:.2f}")
            
            result = self.nav_controller.move_to_position(
                pose.position.x,
                pose.position.y,
                pose.orientation if hasattr(pose, 'orientation') else None
            )
            
            if result:
                print(f"Successfully navigated to {pose_name}")
            else:
                print(f"Failed to navigate to {pose_name}")
                
                # Ask whether to continue
                choice = input("Continue to next pose? (y/n): ").strip().lower()
                if choice != 'y':
                    print("Sequential navigation aborted")
                    return
        
        print("\nCompleted sequential navigation to all poses")
    
    def run(self):
        """Run the node"""
        rospy.spin()
        
if __name__ == '__main__':
    try:
        node = TestNavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test navigation node interrupted")