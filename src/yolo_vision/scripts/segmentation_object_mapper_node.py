#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, ColorRGBA
from sensor_msgs.msg import CameraInfo
import numpy as np

class SegmentationMapperNode:
    """
    Maps segmented objects with depth information to 3D positions in the world.
    
    This node takes segmentation-based object detection data with depth information,
    converts the center of each segmented object to 3D points in the camera frame,
    transforms them to the map frame, and publishes visualization markers at
    the transformed positions.
    """
    def __init__(self):
        rospy.init_node('segmentation_mapper_node', anonymous=True)
        
        # Load parameters
        self.camera_frame = rospy.get_param('~camera_frame')
        self.map_frame = rospy.get_param('~map_frame')
        self.distance_topic = rospy.get_param('~distance_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.object_marker_topic = rospy.get_param('~object_marker_topic')
        self.marker_array_topic = rospy.get_param('~marker_array_topic')
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # State variables
        self.objects = []
        self.camera_info = None
        self.marker_id = 0
        self.class_colors = {}  # Store consistent colors for each class

        # setup communication
        self._setup_ros_communication()
        
        rospy.loginfo("Segmentation mapper node initialized")

    # ---------- Main ROS Communication ----------
        
    def _setup_ros_communication(self):
        # Publishers
        self.marker_array_pub = rospy.Publisher(self.marker_array_topic, MarkerArray, queue_size=10)
        
        # Subscribers
        self.distance_sub = rospy.Subscriber(self.distance_topic, Float32MultiArray, self.distance_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        

    # ---------- Callback Functions ----------

    def camera_info_callback(self, msg):
        """
        Store camera information for later use.
        
        Args:
            msg: CameraInfo message containing camera parameters
        """
        self.camera_info = msg

    def distance_callback(self, msg):
        """
        Process incoming segmentation distance data.
        
        Args:
            msg (std_msgs.msg.Float32MultiArray): Message containing the distances data
                Format: [class_id1, confidence1, depth1, center_x1, center_y1, class_id2, ...]
        """
        data = msg.data
        
        # Check if camera info is available
        if self.camera_info is None:
            rospy.logwarn("Camera info not available yet. Skipping object mapping.")
            return
        

        
        # Create a new marker array
        marker_array = MarkerArray()

        # Add a deletion marker to clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "segmented_objects"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
            
        # Process each object's data (every 5 elements)
        for i in range(0, len(data), 5):
            if i+4 < len(data):  # Ensure we have a complete quintuple
                class_id = int(data[i])
                confidence = data[i+1]
                distance = data[i+2]
                center_x = data[i+3]  # Pixel X coordinate
                center_y = data[i+4]  # Pixel Y coordinate
                
                # Project 2D point to 3D using depth
                camera_point = self.project_pixel_to_3d(center_x, center_y, distance)
                
                # Transform point to map frame
                map_point = self.transform_point_to_map(camera_point)
                
                # Create marker and add to array
                marker = self.create_object_marker(map_point, class_id, confidence)
                marker_array.markers.append(marker)

                if self.debug_mode:
                
                    rospy.loginfo(f"Mapped object: class={class_id}, at position: x={map_point.x:.2f}, y={map_point.y:.2f}, z={map_point.z:.2f}")
        
        # Publish marker array
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

    def project_pixel_to_3d(self, u, v, depth):
        """
        Project a pixel (u,v) with known depth to a 3D point in camera frame.
        
        Args:
            u (float): x coordinate in pixels
            v (float): y coordinate in pixels
            depth (float): depth in meters
            
        Returns:
            Point: 3D point in camera frame
        """
        if self.camera_info is None:
            rospy.logwarn("No camera info available for projection")
            return Point(0, 0, 0)
        
        # Get camera parameters
        fx = self.camera_info.K[0]  # Focal length x
        fy = self.camera_info.K[4]  # Focal length y
        cx = self.camera_info.K[2]  # Principal point x
        cy = self.camera_info.K[5]  # Principal point y
        
        # Project to 3D (pinhole camera model)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return Point(x, y, z)

    def transform_point_to_map(self, point_camera):
        """
        Transform a point from camera frame to map frame.
        
        Args:
            point_camera (Point): Point in camera frame
            
        Returns:
            Point: Point transformed to map frame
        """
        try:
            # Create a PoseStamped in camera frame
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = self.camera_frame
            pose_camera.header.stamp = rospy.Time.now()
            pose_camera.pose.position = point_camera
            pose_camera.pose.orientation.w = 1.0  # Identity rotation
            
            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
            return pose_map.pose.position
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return Point(0, 0, 0)
        
    def get_color_for_class(self, class_id):
        """
        Generate a consistent color for a class ID.
        
        Args:
            class_id: Class ID
            
        Returns:
            ColorRGBA: ROS color object
        """
        # Generate colors based on class ID for consistency
        colors = [
            (0.0, 0.0, 1.0, 1.0),  # Blue
            (0.0, 1.0, 0.0, 1.0),  # Green
            (1.0, 1.0, 0.0, 1.0),  # Yellow

        ]
        
        # Use modulo to handle more classes than colors
        color_tuple = colors[class_id % len(colors)]
        
        # Create and return a ColorRGBA object
        color = ColorRGBA()
        color.r = color_tuple[0]
        color.g = color_tuple[1]
        color.b = color_tuple[2]
        color.a = color_tuple[3]
        
        return color
    

    def create_object_marker(self, position, class_id, confidence):
        """
        Create a marker for an object at the specified position.
        
        Args:
            position (Point): 3D position in map frame
            class_id (int): Object class ID
            confidence (float): Detection confidence
            
        Returns:
            Marker: Visualization marker
        """
        # Create a marker
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "segmented_objects"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        
        # Size based on confidence
        size = 0.1 + (confidence * 0.1)  # Scale size by confidence
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Color based on class
        marker.color = self.get_color_for_class(class_id)
        
        return marker
        


if __name__ == '__main__':
    try:
        node = SegmentationMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass