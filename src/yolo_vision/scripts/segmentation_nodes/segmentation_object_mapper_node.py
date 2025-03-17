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
        self.marker_array_topic = rospy.get_param('~marker_array_topic', '~object_markers')
        self.marker_lifetime = rospy.get_param('~marker_lifetime', 1.0)
        self.show_text_labels = rospy.get_param('~show_text_labels', True)
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.marker_pub = rospy.Publisher(self.object_marker_topic, Marker, queue_size=10)
        self.marker_array_pub = rospy.Publisher(self.marker_array_topic, MarkerArray, queue_size=10)
        
        # Subscribers
        self.distance_sub = rospy.Subscriber(self.distance_topic, Float32MultiArray, self.distance_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # State variables
        self.camera_info = None
        self.marker_id = 0
        self.class_colors = {}  # Store consistent colors for each class
        
        rospy.loginfo("Segmentation mapper node initialized")

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
        Process segmentation-based object detection data with depth information.
        
        Args:
            msg: Float32MultiArray containing 
                 [object_id, class_id, confidence, primary_depth, avg_depth, 
                  median_depth, min_depth, max_depth, valid_percentage, ...]
        """
        try:
            if not msg.data or len(msg.data) % 9 != 0:
                rospy.logwarn(f"Invalid data format: Expected multiple of 9 elements, got {len(msg.data)}")
                return
            
            # Process multiple objects in a single message
            object_count = len(msg.data) // 9
            markers = MarkerArray()
            
            for i in range(object_count):
                # Extract object data: 9 values per object
                start_idx = i * 9
                object_data = msg.data[start_idx:start_idx + 9]
                
                object_id = int(object_data[0])
                class_id = int(object_data[1])
                confidence = object_data[2]
                primary_depth = object_data[3]  # Use primary depth (best estimate)
                
                # Skip if depth is invalid
                if primary_depth <= 0 or np.isnan(primary_depth):
                    continue
                
                # Create 3D point for object
                # Note: For segmentation-based objects, we don't have explicit 
                # center coordinates. We use the camera optical center and the depth.
                if self.camera_info is None:
                    rospy.logwarn("No camera info received yet")
                    continue
                
                # Use optical center as the point estimation
                # This assumes the segmentation object is centered in the image
                # For more accurate positioning, we would need the centroid coordinates
                center_x = self.camera_info.width / 2
                center_y = self.camera_info.height / 2
                
                # Convert to 3D point in camera frame
                point_3d = self.pixel_to_3d(center_x, center_y, primary_depth)
                if point_3d is None:
                    continue
                
                # Create pose in camera frame
                camera_pose = self.create_camera_frame_pose(point_3d)
                
                # Transform to map frame
                map_pose = self.transform_to_map_frame(camera_pose)
                if map_pose is not None:
                    # Create marker for this object
                    marker = self.create_marker(map_pose, class_id, confidence, object_id)
                    
                    # Add to single marker publisher
                    self.marker_pub.publish(marker)
                    
                    # Add to marker array
                    markers.markers.append(marker)
                    
                    # Create text marker if enabled
                    if self.show_text_labels:
                        text_marker = self.create_text_marker(map_pose, class_id, confidence, object_id)
                        markers.markers.append(text_marker)
            
            # Publish all markers at once
            if markers.markers:
                self.marker_array_pub.publish(markers)
                
        except Exception as e:
            rospy.logerr(f"Error in distance_callback: {str(e)}")

    # ---------- Coordinate Transformation Functions ----------

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

    def get_color_for_class(self, class_id):
        """
        Get a consistent color for a class ID.
        
        Args:
            class_id: Integer class ID
            
        Returns:
            ColorRGBA: Color for the class
        """
        if class_id not in self.class_colors:
            # Generate deterministic color based on class ID
            r = ((class_id * 47) % 255) / 255.0
            g = ((class_id * 97) % 255) / 255.0
            b = ((class_id * 193) % 255) / 255.0
            self.class_colors[class_id] = ColorRGBA(r, g, b, 1.0)
        
        return self.class_colors[class_id]

    def create_marker(self, pose, class_id, confidence, object_id):
        """
        Create a marker for visualization.
        
        Args:
            pose: PoseStamped message to place the marker
            class_id: Integer class ID
            confidence: Detection confidence score
            object_id: Object identifier
            
        Returns:
            Marker: Visualization marker
        """
        marker = Marker()
        marker.header = pose.header
        marker.ns = "segmented_objects"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = pose.pose
        
        # Scale marker based on confidence
        scale = 0.1 + (confidence * 0.1)  # Scale between 0.1 and 0.2 based on confidence
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        
        # Color based on class ID
        marker.color = self.get_color_for_class(class_id)
        
        # Set marker lifetime
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        
        return marker

    def create_text_marker(self, pose, class_id, confidence, object_id):
        """
        Create a text marker to label the object.
        
        Args:
            pose: PoseStamped message to place the marker
            class_id: Integer class ID
            confidence: Detection confidence score
            object_id: Object identifier
            
        Returns:
            Marker: Text marker
        """
        marker = Marker()
        marker.header = pose.header
        marker.ns = "object_labels"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position text above the object
        marker.pose = pose.pose
        marker.pose.position.z += 0.2  # Offset above object
        
        # Scale text based on distance
        marker.scale.z = 0.1  # Text height
        
        # Use same color as the object
        marker.color = self.get_color_for_class(class_id)
        
        # Create label with class ID and confidence
        marker.text = f"Class:{class_id} ({confidence:.2f})"
        
        # Set marker lifetime
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        
        return marker


if __name__ == '__main__':
    try:
        node = SegmentationMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass