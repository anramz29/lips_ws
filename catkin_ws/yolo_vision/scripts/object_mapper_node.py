#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo
import numpy as np

class ObjectMapperNode:
    def __init__(self):
        rospy.init_node('object_mapper_node', anonymous=True)
        
        # Load parameters
        self.camera_frame = rospy.get_param('~camera_frame')
        self.map_frame = rospy.get_param('~map_frame')
        self.bbox_depth_topic = rospy.get_param('~bbox_depth_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.object_marker_topic = rospy.get_param('~object_marker_topic')
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.marker_pub = rospy.Publisher(self.object_marker_topic, Marker, queue_size=10)
        
        # Subscribers
        self.bbox_sub = rospy.Subscriber(self.bbox_depth_topic, Float32MultiArray, self.bbox_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        self.camera_info = None

    def camera_info_callback(self, msg):
        """Store camera information for later use."""
        self.camera_info = msg

    def pixel_to_3d(self, u, v, depth):
        """Convert pixel coordinates and depth to 3D camera coordinates."""
        if self.camera_info is None:
            rospy.logwarn("No camera info received yet")
            return None

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        # Convert pixel coordinates to 3D point
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return Point(x, y, z)

    def bbox_callback(self, msg):
        """Process bounding box and depth information."""
        try:
            # Extract bbox information
            x, y, w, h, depth = msg.data
            
            # Calculate center of bounding box
            center_x = x + w/2
            center_y = y + h/2
            
            # Convert to 3D point in camera frame
            point_3d = self.pixel_to_3d(center_x, center_y, depth)
            if point_3d is None:
                return

            # Create PoseStamped message
            pose = PoseStamped()
            pose.header.frame_id = self.camera_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = point_3d
            pose.pose.orientation.w = 1.0

            try:
                # Transform point from camera frame to map frame
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)

                # Create and publish marker
                self.publish_marker(pose_transformed)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {str(e)}")

        except Exception as e:
            rospy.logerr(f"Error in bbox_callback: {str(e)}")

    def publish_marker(self, pose):
        """Publish visualization marker at the transformed position."""
        marker = Marker()
        marker.header = pose.header
        marker.ns = "detected_objects"
        marker.id = 0
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = pose.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(1)  # 0 means forever
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        node = ObjectMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass