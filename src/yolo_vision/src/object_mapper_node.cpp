#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <string>
#include <memory>

/**
 * Maps 2D bounding boxes with depth information to 3D positions in the world.
 *
 * This node takes bounding box coordinates and depth information, converts them
 * to 3D points in the camera frame, transforms them to the map frame, and
 * publishes visualization markers at the transformed positions.
 */
class ObjectMapperNode {
public:
    ObjectMapperNode() : nh_("~") {
        // Load parameters
        nh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "/distance/bbox_depth");
        nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
        nh_.param<std::string>("object_marker_topic", object_marker_topic_, "/object_markers");

        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(object_marker_topic_, 10);

        // Subscribers
        bbox_sub_ = nh_.subscribe(bbox_depth_topic_, 10, &ObjectMapperNode::bboxCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 10, &ObjectMapperNode::cameraInfoCallback, this);

        ROS_INFO("Object mapper node initialized");
    }

    void run() {
        ros::spin();
    }

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber camera_info_sub_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string camera_frame_;
    std::string map_frame_;
    std::string bbox_depth_topic_;
    std::string camera_info_topic_;
    std::string object_marker_topic_;

    // State variables
    sensor_msgs::CameraInfo::ConstPtr camera_info_;

    /**
     * Store camera information for later use.
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        camera_info_ = msg;
    }

    /**
     * Process bounding box and depth information.
     *
     * Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
     */
    void bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        try {
            // Extract data and create 3D point
            auto [point_3d, depth] = bboxTo3DPoint(msg->data);
            if (!point_3d) {
                return;
            }

            // Create pose in camera frame
            geometry_msgs::PoseStamped camera_pose = createCameraFramePose(*point_3d);

            // Transform to map frame
            auto map_pose = transformToMapFrame(camera_pose);
            if (map_pose) {
                // Create and publish marker
                publishMarker(*map_pose);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in bboxCallback: " << e.what());
        }
    }

    /**
     * Convert bounding box data to a 3D point in camera coordinates.
     *
     * Format: [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
     *
     * @return std::pair containing optional Point and depth
     */
    std::pair<std::optional<geometry_msgs::Point>, float> bboxTo3DPoint(const std::vector<float>& bbox_data) {
        // Log the actual data for debugging
        ROS_DEBUG("Received bbox_data size: %zu", bbox_data.size());

        // Extract the fields from the format
        if (bbox_data.empty()) {
            return {std::nullopt, 0.0f};
        }

        int n_boxes = static_cast<int>(bbox_data[0]);
        if (n_boxes < 1) {
            return {std::nullopt, 0.0f};
        }

        int cls_id = static_cast<int>(bbox_data[1]);
        float confidence = bbox_data[2];
        float x1 = bbox_data[3];
        float y1 = bbox_data[4];
        float x2 = bbox_data[5];
        float y2 = bbox_data[6];
        float depth = bbox_data[7];

        // Calculate center point from bbox corners
        float center_x = (x1 + x2) / 2.0f;
        float center_y = (y1 + y2) / 2.0f;

        // Log the extracted values for debugging
        ROS_DEBUG("Extracted values: boxes=%d, class=%d, conf=%.2f, depth=%.2f, center=(%.1f, %.1f)",
                  n_boxes, cls_id, confidence, depth, center_x, center_y);

        // Convert to 3D point in camera frame
        auto point_3d = pixelTo3D(center_x, center_y, depth);
        return {point_3d, depth};
    }

    /**
     * Convert pixel coordinates and depth to 3D camera coordinates.
     *
     * @param u Pixel x-coordinate
     * @param v Pixel y-coordinate
     * @param depth Depth in meters
     * @return Optional 3D point in camera coordinates
     */
    std::optional<geometry_msgs::Point> pixelTo3D(float u, float v, float depth) {
        if (!camera_info_) {
            ROS_WARN("No camera info received yet");
            return std::nullopt;
        }

        // Extract camera intrinsics
        float fx = camera_info_->K[0];
        float fy = camera_info_->K[4];
        float cx = camera_info_->K[2];
        float cy = camera_info_->K[5];

        // Convert pixel coordinates to 3D point
        geometry_msgs::Point point;
        point.x = (u - cx) * depth / fx;
        point.y = (v - cy) * depth / fy;
        point.z = depth;

        return point;
    }

    /**
     * Create a PoseStamped message in the camera frame.
     *
     * @param point_3d 3D point in camera coordinates
     * @return PoseStamped in camera frame
     */
    geometry_msgs::PoseStamped createCameraFramePose(const geometry_msgs::Point& point_3d) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = camera_frame_;
        pose.header.stamp = ros::Time::now();
        pose.pose.position = point_3d;
        pose.pose.orientation.w = 1.0;  // Identity quaternion (no rotation)

        return pose;
    }

    /**
     * Transform a pose from camera frame to map frame.
     *
     * @param camera_pose PoseStamped in camera frame
     * @return Optional transformed pose in map frame
     */
    std::optional<geometry_msgs::PoseStamped> transformToMapFrame(const geometry_msgs::PoseStamped& camera_pose) {
        try {
            // Look up transform from camera to map
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                map_frame_,
                camera_frame_,
                ros::Time(0),
                ros::Duration(1.0)
            );

            // Apply transform to pose
            geometry_msgs::PoseStamped pose_transformed;
            tf2::doTransform(camera_pose, pose_transformed, transform);
            return pose_transformed;
        } catch (const tf2::TransformException& e) {
            ROS_WARN_STREAM("TF Error: " << e.what());
            return std::nullopt;
        }
    }

    /**
     * Publish visualization marker at the transformed position.
     *
     * @param pose PoseStamped message in map frame
     */
    void publishMarker(const geometry_msgs::PoseStamped& pose) {
        auto marker = createMarker(pose);
        marker_pub_.publish(marker);
    }

    /**
     * Create a marker for visualization.
     *
     * @param pose PoseStamped message to place the marker
     * @return Visualization marker
     */
    visualization_msgs::Marker createMarker(const geometry_msgs::PoseStamped& pose) {
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

        marker.lifetime = ros::Duration(1);  // 1 second lifetime

        return marker;
    }
};

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "object_mapper_node");
        ObjectMapperNode node;
        node.run();
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("ROS error: " << e.what());
        return 1;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error: " << e.what());
        return 1;
    }

    return 0;
}