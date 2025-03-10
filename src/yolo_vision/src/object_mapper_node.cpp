#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>

class ObjectMapperNode
{
public:
    ObjectMapperNode() : private_nh_("~"), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_)
    {
        // Load parameters
        private_nh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");
        private_nh_.param<std::string>("map_frame", map_frame_, "map");
        private_nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "bbox_depth");
        private_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "camera_info");
        private_nh_.param<std::string>("object_marker_topic", object_marker_topic_, "object_markers");

        // Publishers
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(object_marker_topic_, 10);

        // Subscribers
        bbox_sub_ = nh_.subscribe(bbox_depth_topic_, 10, &ObjectMapperNode::bboxCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &ObjectMapperNode::cameraInfoCallback, this);

        camera_info_received_ = false;
    }

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers and subscribers
    ros::Publisher marker_pub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber camera_info_sub_;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string camera_frame_;
    std::string map_frame_;
    std::string bbox_depth_topic_;
    std::string camera_info_topic_;
    std::string object_marker_topic_;

    // Camera info
    sensor_msgs::CameraInfo camera_info_;
    bool camera_info_received_;

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        camera_info_ = *msg;
        camera_info_received_ = true;
    }

    geometry_msgs::Point pixelTo3D(float u, float v, float depth)
    {
        geometry_msgs::Point point;

        if (!camera_info_received_) {
            ROS_WARN("No camera info received yet");
            return point;
        }

        // Extract camera parameters
        float fx = camera_info_.K[0];
        float fy = camera_info_.K[4];
        float cx = camera_info_.K[2];
        float cy = camera_info_.K[5];

        // Convert pixel coordinates to 3D point
        point.x = (u - cx) * depth / fx;
        point.y = (v - cy) * depth / fy;
        point.z = depth;

        return point;
    }

    void bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        try {
            // Check if we have enough data
            if (msg->data.size() < 5) {
                ROS_WARN("Received bbox data with insufficient elements");
                return;
            }

            // Extract bbox information
            float x = msg->data[0];
            float y = msg->data[1];
            float w = msg->data[2];
            float h = msg->data[3];
            float depth = msg->data[4];

            // Calculate center of bounding box
            float center_x = x + w/2;
            float center_y = y + h/2;

            // Convert to 3D point in camera frame
            geometry_msgs::Point point_3d = pixelTo3D(center_x, center_y, depth);

            // Create PoseStamped message
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = camera_frame_;
            pose.header.stamp = ros::Time::now();
            pose.pose.position = point_3d;
            pose.pose.orientation.w = 1.0;

            try {
                // Transform point from camera frame to map frame
                geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                    map_frame_,
                    camera_frame_,
                    ros::Time(0),
                    ros::Duration(1.0)
                );

                geometry_msgs::PoseStamped pose_transformed;
                tf2::doTransform(pose, pose_transformed, transform);

                // Create and publish marker
                publishMarker(pose_transformed);

            } catch (tf2::TransformException &ex) {
                ROS_WARN_STREAM("TF Error: " << ex.what());
            }

        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in bboxCallback: " << e.what());
        }
    }

    void publishMarker(const geometry_msgs::PoseStamped& pose)
    {
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
        
        marker.lifetime = ros::Duration(1);  // 0 means forever
        
        marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_mapper_node");
    ObjectMapperNode node;
    ros::spin();
    return 0;
}