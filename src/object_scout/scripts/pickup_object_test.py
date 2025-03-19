#!/usr/bin/env python3

import rospy
from object_scoute.pick_up_object import PickUpObject  # Assuming you saved the class in a file named pick_up_object.py

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('pick_up_object_standalone', anonymous=False)
    
    try:
        # Create instance of the PickUpObject class
        pickup = PickUpObject(robot_model="locobot_wx250s", init_node=False)
        
        # Run the pick operation
        pickup.pick_object()
        
        # Keep the node running until shutdown
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Error occurred: {e}")