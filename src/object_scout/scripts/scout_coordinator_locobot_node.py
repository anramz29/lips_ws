#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg
import time

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

from object_scout.scout_coordinator_locobot import ScoutCoordinatorLocobot 

def prompt_user():
    """Prompt the user to start the mission"""
    print("\n===== Object Scout Mission =====")
    print("This will start the robot's object scouting mission.")
    print("The robot will navigate to predefined poses and look for objects.")
    
    while True:
        response = input("\nDo you want to start the mission? (yes/no): ").strip().lower()
        if response in ['yes', 'y']:
            return True
        elif response in ['no', 'n']:
            return False
        else:
            print("Please enter 'yes' or 'no'.")

def main():
    # Create the coordinator without starting the mission yet
    coordinator = ScoutCoordinatorLocobot(init_node=True)
    
    # Log startup
    rospy.loginfo("=== Object Scout Coordinator Node Started ===")

    rospy.sleep(5)  # Wait for other nodes to start
    
    # Prompt user to start the mission
    if prompt_user():
        rospy.loginfo("Starting mission as requested by user...")
        # Add a small delay for better user experience
        time.sleep(1)
        
        # Run the main mission
        coordinator.execute_scouting_mission()
    else:
        rospy.loginfo("Mission start cancelled by user")
    
    # Keep the node running
    rospy.loginfo("Scout coordinator is now idle. Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator node interrupted")
    except Exception as e:
        rospy.logerr(f"Scout coordinator node encountered an error: {e}")
        # Try to safely stop the robot
        try:
            coordinator = ScoutCoordinatorLocobot(init_node=False)
            coordinator.shutdown_handler()
        except:
            pass