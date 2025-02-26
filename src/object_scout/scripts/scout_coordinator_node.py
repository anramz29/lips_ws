#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

from object_scout.scout_coordinator import ScoutCoordinator

def main():
    # Create the coordinator
    coordinator = ScoutCoordinator(init_node=True)
    
    # Log startup
    rospy.loginfo("=== Object Scout Coordinator Node Started ===")
    
    # Run the main mission
    result = coordinator.move_to_all_poses()
    
    if result:
        rospy.loginfo("Successfully completed mission and found an object!")
    else:
        rospy.loginfo("Completed mission, no objects approached")
        
    # Keep the node running
    if not result:
        rospy.spin()  # Keep node running if no object was found

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scout coordinator node interrupted")