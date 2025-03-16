#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo
import numpy as np

class ObjectMapperNode:
    """
    Maps 2D bounding boxes with depth information to 3D positions in the world.
    
    This node takes bounding box coordinates and depth information, converts them
    to 3D points in the camera frame, transforms them to the map frame, and
    publishes visualization markers at the transformed positions.
    """
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
        
        # State variables
        self.camera_info = None
        
        rospy.loginfo("Object mapper node initialized")

    # ---------- Callback Functions ----------

    def camera_info_callback(self, msg):
        """
        Store camera information for later use.
        
        Args:
            msg: CameraInfo message containing camera parameters
        """
        self.camera_info = msg

    def bbox_callback(self, msg):
        """
        Process bounding box and depth information.
        
        Args:
            msg: Float32MultiArray containing [x, y, width, height, depth]
        """
        try:
            # Extract data and create 3D point
            point_3d, depth = self.bbox_to_3d_point(msg.data)
            if point_3d is None:
                return

            # Create pose in camera frame
            camera_pose = self.create_camera_frame_pose(point_3d)
            
            # Transform to map frame
            map_pose = self.transform_to_map_frame(camera_pose)
            if map_pose is not None:
                # Create and publish marker
                self.publish_marker(map_pose)

        except Exception as e:
            rospy.logerr(f"Error in bbox_callback: {str(e)}")

    # ---------- Coordinate Transformation Functions ----------

    def bbox_to_3d_point(self, bbox_data):
        """
        Convert bounding box data to a 3D point in camera coordinates.
        
        Args:
            bbox_data: List of [x, y, width, height, depth]
            
        Returns:
            tuple: (Point, depth) or (None, None) if conversion fails
        """
        if len(bbox_data) < 5:
            rospy.logwarn("Bbox data has insufficient elements")
            return None, None
            
        x, y, w, h, depth = bbox_data
        
        # Calculate center of bounding box
        center_x = x + w/2
        center_y = y + h/2
        
        # Convert to 3D point in camera frame
        point_3d = self.pixel_to_3d(center_x, center_y, depth)
        return point_3d, depth

    def pixel_to_3d(self, u, v, depth):
        """
        Convert pixel coordinates and depth to 3D camera coordinates.
        
        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate
            depth: Depth in meters
            
        Returns:
            Point: 3D point in camera coordinates or None if conversion fails
        """
        if self.camera_info is None:
            rospy.logwarn("No camera info received yet")
            return None

        # Extract camera intrinsics
        fx = self.camera_info.K[0] 
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        # Convert pixel coordinates to 3D point
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return Point(x, y, z)

    def create_camera_frame_pose(self, point_3d):
        """
        Create a PoseStamped message in the camera frame.
        
        Args:
            point_3d: 3D point in camera coordinates
            
        Returns:
            PoseStamped: Pose in camera frame
        """
        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = point_3d
        pose.pose.orientation.w = 1.0  # Identity quaternion (no rotation)
        
        return pose

    def transform_to_map_frame(self, camera_pose):
        """
        Transform a pose from camera frame to map frame.
        
        Args:
            camera_pose: PoseStamped in camera frame
            
        Returns:
            PoseStamped: Transformed pose in map frame or None if transformation fails
        """
        try:
            # Look up transform from camera to map
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Apply transform to pose
            pose_transformed = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)
            return pose_transformed
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {str(e)}")
            return None

    # ---------- Visualization Functions ----------

    def publish_marker(self, pose):
        """
        Publish visualization marker at the transformed position.
        
        Args:
            pose: PoseStamped message in map frame
        """
        marker = self.create_marker(pose)
        self.marker_pub.publish(marker)

    def create_marker(self, pose):
        """
        Create a marker for visualization.
        
        Args:
            pose: PoseStamped message to place the marker
            
        Returns:
            Marker: Visualization marker
        """
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
        
        marker.lifetime = rospy.Duration(1)  # 1 second lifetime
        
        return marker


if __name__ == '__main__':
    try:
        node = ObjectMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass