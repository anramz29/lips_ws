#ifndef OBJECT_MAPPER_NODE_HPP
#define OBJECT_MAPPER_NODE_HPP

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

/**
 * @class ObjectMapperNode
 * @brief Maps 2D bounding boxes with depth information to 3D positions in the world.
 * 
 * This node takes bounding box coordinates and depth information, converts them
 * to 3D points in the camera frame, transforms them to the map frame, and
 * publishes visualization markers at the transformed positions.
 */
class ObjectMapperNode {
public:
    /**
     * @brief Constructor
     */
    ObjectMapperNode();
    
    /**
     * @brief Run the node
     */
    void spin();

private:
    // ---------- Callback Functions ----------
    
    /**
     * @brief Store camera information for later use
     * @param msg CameraInfo message containing camera parameters
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    
    /**
     * @brief Process bounding box and depth information
     * @param msg Float32MultiArray containing [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
     */
    void bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    // ---------- Coordinate Transformation Functions ----------
    
    /**
     * @brief Convert bounding box data to a 3D point in camera coordinates
     * @param bboxData Array from YOLO detection [n_boxes, cls_id, conf, x1, y1, x2, y2, depth]
     * @param depth Output parameter to store depth value
     * @return 3D point in camera coordinates or null if conversion fails
     */
    geometry_msgs::Point bboxTo3DPoint(const std::vector<float>& bboxData, float& depth);
    
    /**
     * @brief Convert pixel coordinates and depth to 3D camera coordinates
     * @param u Pixel x-coordinate
     * @param v Pixel y-coordinate
     * @param depth Depth in meters
     * @return 3D point in camera coordinates or null if conversion fails
     */
    geometry_msgs::Point pixelTo3D(float u, float v, float depth);
    
    /**
     * @brief Create a PoseStamped message in the camera frame
     * @param point3D 3D point in camera coordinates
     * @return Pose in camera frame
     */
    geometry_msgs::PoseStamped createCameraFramePose(const geometry_msgs::Point& point3D);
    
    /**
     * @brief Transform a pose from camera frame to map frame
     * @param cameraPose PoseStamped in camera frame
     * @return Transformed pose in map frame or null if transformation fails
     */
    geometry_msgs::PoseStamped transformToMapFrame(const geometry_msgs::PoseStamped& cameraPose);
    
    // ---------- Visualization Functions ----------
    
    /**
     * @brief Publish visualization marker at the transformed position
     * @param pose PoseStamped message in map frame
     */
    void publishMarker(const geometry_msgs::PoseStamped& pose);
    
    /**
     * @brief Create a marker for visualization
     * @param pose PoseStamped message to place the marker
     * @return Visualization marker
     */
    visualization_msgs::Marker createMarker(const geometry_msgs::PoseStamped& pose);
    
    // ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // ROS parameters
    std::string camera_frame_;
    std::string map_frame_;
    std::string bbox_depth_topic_;
    std::string camera_info_topic_;
    std::string object_marker_topic_;
    
    // TF2 
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Publishers
    ros::Publisher marker_pub_;
    
    // Subscribers
    ros::Subscriber bbox_sub_;
    ros::Subscriber camera_info_sub_;
    
    // State variables
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    bool has_camera_info_;
};

#endif // OBJECT_MAPPER_NODE_HPP