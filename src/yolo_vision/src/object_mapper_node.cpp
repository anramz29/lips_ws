#include "yolo_vision/object_mapper_node.hpp"

ObjectMapperNode::ObjectMapperNode() 
    : private_nh_("~"), 
      tf_listener_(tf_buffer_),
      has_camera_info_(false) {
    
    // Load parameters
    private_nh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "/distance_node/bbox_depth");
    private_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
    private_nh_.param<std::string>("object_marker_topic", object_marker_topic_, "/object_markers");
    
    // Publishers
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(object_marker_topic_, 10);
    
    // Subscribers
    bbox_sub_ = nh_.subscribe(bbox_depth_topic_, 10, &ObjectMapperNode::bboxCallback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &ObjectMapperNode::cameraInfoCallback, this);
    
    ROS_INFO("Object mapper node initialized");
}

// ---------- Callback Functions ----------

void ObjectMapperNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_info_ = msg;
    has_camera_info_ = true;
}

void ObjectMapperNode::bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    try {
        // Check if data is available
        if (msg->data.empty()) {
            ROS_WARN("Empty bbox data received");
            return;
        }
        
        // Extract data and create 3D point
        float depth;
        std::vector<float> data(msg->data.begin(), msg->data.end());
        geometry_msgs::Point point3D = bboxTo3DPoint(data, depth);
        
        // Check if conversion was successful
        if (point3D.x == 0 && point3D.y == 0 && point3D.z == 0 && depth == 0) {
            return;
        }

        // Create pose in camera frame
        geometry_msgs::PoseStamped cameraPose = createCameraFramePose(point3D);
        
        // Transform to map frame
        geometry_msgs::PoseStamped mapPose = transformToMapFrame(cameraPose);
        
        // Check if transformation was successful
        if (mapPose.header.frame_id == map_frame_) {
            // Create and publish marker
            publishMarker(mapPose);
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error in bboxCallback: " << e.what());
    }
}

// ---------- Coordinate Transformation Functions ----------

geometry_msgs::Point ObjectMapperNode::bboxTo3DPoint(const std::vector<float>& bboxData, float& depth) {
    // Log the actual data for debugging
    ROS_DEBUG_STREAM("Received bbox_data: [" << bboxData.size() << " elements]");
    
    // Default return value
    geometry_msgs::Point point;
    depth = 0.0;
    
    // Check if we have enough data
    if (bboxData.size() < 8) {
        ROS_WARN_STREAM("Invalid bbox data size: " << bboxData.size());
        return point;
    }
    
    // Extract the fields from the data
    int n_boxes = static_cast<int>(bboxData[0]);
    if (n_boxes < 1) {
        ROS_DEBUG("No boxes detected");
        return point;
    }
    
    int cls_id = static_cast<int>(bboxData[1]);
    float confidence = bboxData[2];
    float x1 = bboxData[3], y1 = bboxData[4];
    float x2 = bboxData[5], y2 = bboxData[6];
    depth = bboxData[7];
    
    // Calculate center point from bbox corners
    float center_x = (x1 + x2) / 2.0f;
    float center_y = (y1 + y2) / 2.0f;
    
    // Log the extracted values for debugging
    ROS_DEBUG_STREAM("Extracted values: boxes=" << n_boxes 
                    << ", class=" << cls_id 
                    << ", conf=" << confidence 
                    << ", depth=" << depth 
                    << ", center=(" << center_x << ", " << center_y << ")");
    
    // Convert to 3D point in camera frame
    return pixelTo3D(center_x, center_y, depth);
}

geometry_msgs::Point ObjectMapperNode::pixelTo3D(float u, float v, float depth) {
    geometry_msgs::Point point;
    
    // Default initialization
    point.x = 0;
    point.y = 0;
    point.z = 0;
    
    if (!has_camera_info_) {
        ROS_WARN("No camera info received yet");
        return point;
    }
    
    if (depth <= 0.0f) {
        ROS_DEBUG("Invalid depth value: %f", depth);
        return point;
    }

    // Extract camera intrinsics
    float fx = camera_info_->K[0];
    float fy = camera_info_->K[4];
    float cx = camera_info_->K[2];
    float cy = camera_info_->K[5];

    // Convert pixel coordinates to 3D point
    point.x = (u - cx) * depth / fx;
    point.y = (v - cy) * depth / fy;
    point.z = depth;

    return point;
}

geometry_msgs::PoseStamped ObjectMapperNode::createCameraFramePose(const geometry_msgs::Point& point3D) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = camera_frame_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position = point3D;
    pose.pose.orientation.w = 1.0;  // Identity quaternion (no rotation)
    
    return pose;
}

geometry_msgs::PoseStamped ObjectMapperNode::transformToMapFrame(const geometry_msgs::PoseStamped& cameraPose) {
    try {
        // Look up transform from camera to map
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            map_frame_,
            camera_frame_,
            ros::Time(0),
            ros::Duration(1.0)
        );
        
        // Apply transform to pose
        geometry_msgs::PoseStamped poseTransformed;
        tf2::doTransform(cameraPose, poseTransformed, transform);
        return poseTransformed;
    }
    catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM("TF Error: " << e.what());
        // Return an empty pose with an empty frame_id to indicate failure
        geometry_msgs::PoseStamped emptyPose;
        return emptyPose;
    }
}

// ---------- Visualization Functions ----------

void ObjectMapperNode::publishMarker(const geometry_msgs::PoseStamped& pose) {
    visualization_msgs::Marker marker = createMarker(pose);
    marker_pub_.publish(marker);
}

visualization_msgs::Marker ObjectMapperNode::createMarker(const geometry_msgs::PoseStamped& pose) {
    visualization_msgs::Marker marker;
    marker.header = pose.header;
    marker.ns = "detected_objects";
    marker.id = 0;
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose = pose.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration(1.0);  // 1 second lifetime
    
    return marker;
}

void ObjectMapperNode::spin() {
    // Start processing in ROS event loop
    ros::spin();
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "object_mapper_node");
    
    // Create node instance
    ObjectMapperNode node;
    
    // Run the node
    node.spin();
    
    return 0;
}