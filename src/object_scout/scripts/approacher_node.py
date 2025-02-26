#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg
from visualization_msgs.msg import Marker

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from src.object_scout.navigation_controller import NavigationController
from src.object_scout.object_approacher import ObjectApproacher

# Custom service message imports (uncomment and implement if needed)
# from object_scout.srv import ApproachObject, ApproachObjectResponse

class ApproacherNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('object_approacher')
        
        # Get parameters
        self.robot_name = rospy.get_param('~robot_name', 'locobot')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic', f'/{self.robot_name}/camera/yolo/bbox_depth')
        self.object_marker_topic = rospy.get_param('~object_marker_topic', f'/{self.robot_name}/detected_object/marker')
        
        # Create navigation controller
        self.nav_controller = NavigationController(self.robot_name, init_node=False)
        
        # Create approacher
        self.approacher = ObjectApproacher(self.robot_name, self.nav_controller, init_node=False)
        
        # Set up direct marker subscriber for standalone operation
        self.marker_sub = rospy.Subscriber(
            self.object_marker_topic,
            Marker,
            self.marker_callback
        )
        
        # Set up services
        # Uncomment and implement if needed
        # self.approach_service = rospy.Service('~approach_object', ApproachObject, self.approach_object_cb)
        
        # Log startup
        rospy.loginfo("Object Approacher Node started")
    
    def marker_callback(self, marker_msg):
        """
        Process incoming object markers and approach them if not already approaching
        """
        # Only approach if we're not already approaching an object
        if not hasattr(self, 'is_approaching') or not self.is_approaching:
            rospy.loginfo(f"Received new object marker, initiating approach")
            self.is_approaching = True
            
            # Start approach in a separate thread to not block the callback
            import threading
            threading.Thread(target=self.approach_marker, args=(marker_msg,)).start()
    
    def approach_marker(self, marker):
        """Execute approach on a separate thread"""
        try:
            result = self.approacher.approach_object(marker)
            rospy.loginfo(f"Approach completed with result: {result}")
        finally:
            self.is_approaching = False
    
    def approach_object_cb(self, req):
        """
        Service callback for approach_object service
        
        Note: Uncomment and implement if needed
        """
        # Extract parameters from the request
        # marker = req.marker
        
        # Call the approacher method
        # success = self.approacher.approach_object(marker)
        
        # Return the response
        # return {'success': success}
        pass
        
    def run(self):
        """Run the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ApproacherNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object approacher node interrupted")