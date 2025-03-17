#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

class KeypointActivationNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('keypoint_activation_node', anonymous=True)
        
        # Get robot name parameter
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        
        # Create publisher for enabling/disabling keypoint detection
        self.enable_pub = rospy.Publisher(
            f'/{self.robot_name}/keypoint_detector/enable',
            Bool,
            queue_size=1,
            latch=True  # Make it "sticky" so new subscribers get the last value
        )
        
        # Set initial state (disabled)
        self.enable_pub.publish(Bool(False))
        
        # Create service
        self.service = rospy.Service(
            f'/{self.robot_name}/keypoint_detector/set_enabled',
            SetBool,
            self.handle_set_enabled
        )
        
        rospy.loginfo("Keypoint activation service started")
    
    def handle_set_enabled(self, req):
        """Handle service request to enable/disable keypoint detection"""
        # Publish the requested state
        self.enable_pub.publish(Bool(req.data))
        
        state = "enabled" if req.data else "disabled"
        rospy.loginfo(f"Keypoint detection {state}")
        
        # Return success response
        return SetBoolResponse(
            success=True,
            message=f"Keypoint detection {state}"
        )

if __name__ == '__main__':
    try:
        node = KeypointActivationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass