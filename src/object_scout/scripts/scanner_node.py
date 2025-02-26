#!/usr/bin/env python3
import rospy
import sys
import os
import rospkg

# Ensure the package is in the Python path
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('object_scout')
sys.path.append(os.path.join(pkg_path, 'src'))

from src.object_scout.navigation_controller import NavigationController
from src.object_scout.object_scanner import ObjectScanner

# Custom service message imports (you would need to define these)
# from object_scout.srv import PerformScan, PerformScanResponse

def perform_scan_cb(req):
    """
    Service callback for perform_scan service
    
    Note: You would need to define the appropriate service message
    """
    # Call the scanner method
    result = scanner.perform_scan_rotation()
    
    # Get the detected object if any
    marker = scanner.object_marker if scanner.object_detected else None
    
    # Return the response
    return {
        'success': scanner.object_detected,
        'marker': marker
    }

def main():
    # Initialize the node
    rospy.init_node('object_scanner')
    
    # Get parameters
    robot_name = rospy.get_param('~robot_name', 'locobot')
    
    # First create a navigation controller that scanner depends on
    nav_controller = NavigationController(robot_name, init_node=False)
    
    # Then create the scanner
    global scanner
    scanner = ObjectScanner(robot_name, nav_controller, init_node=False)
    
    # Set up services
    # Note: You would need to define and build these service messages
    # rospy.Service('~perform_scan', PerformScan, perform_scan_cb)
    
    # Log startup
    rospy.loginfo("Object Scanner Node started")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object scanner node interrupted")