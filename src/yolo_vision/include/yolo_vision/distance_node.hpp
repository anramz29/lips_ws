#ifndef DISTANCE_NODE_HPP
#define DISTANCE_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

/**
 * @class DistanceNode
 * @brief Node for calculating distances from depth images using bounding boxes from YOLO.
 * 
 * Subscribes to YOLO bounding boxes and depth images.
 * Publishes annotated RGB images and bounding boxes with depth information.
 */
class DistanceNode {
public:
    /**
     * @brief Constructor
     */
    DistanceNode();
    
    /**
     * @brief Run the node
     */
    void spin();

private:
    /**
     * @brief Setup ROS communication (subscribers, publishers, and timer)
     */
    void setupRosCommunication();
    
    /**
     * @brief Callback for RGB image messages
     * @param msg The received image message
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    /**
     * @brief Callback for depth image messages
     * @param msg The received depth image message
     */
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    
    /**
     * @brief Callback for bounding box messages
     * @param msg The received bounding box message
     */
    void bboxCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    
    /**
     * @brief Process all available data (timer callback)
     * @param event Timer event
     */
    void processData(const ros::TimerEvent& event);
    
    /**
     * @brief Convert ROS depth image to meters in OpenCV format
     * @param rosImage ROS depth image
     * @return Depth image in meters
     */
    cv::Mat convertDepthImageToCv2(const sensor_msgs::ImageConstPtr& rosImage);
    
    /**
     * @brief Calculate average depth for a bounding box
     * @param depth Depth image
     * @param bbox Bounding box (x1, y1, x2, y2)
     * @return Average depth in meters
     */
    float processBboxDepth(const cv::Mat& depth, const cv::Rect& bbox);
    
    /**
     * @brief Process bounding boxes with depth information
     * @param frame RGB frame for annotation
     * @param depthImage Depth image
     * @param bboxData Bounding box data
     * @return Updated bounding box data with depth
     */
    std::vector<float> processBboxesWithDepth(cv::Mat& frame, const cv::Mat& depthImage, 
                                             const std::vector<float>& bboxData);
    
    /**
     * @brief Annotate frame with detection information
     * @param frame Frame to annotate
     * @param clsId Class ID
     * @param conf Confidence
     * @param x1, y1, x2, y2 Bounding box coordinates
     * @param depth Calculated depth
     */
    void annotateFrame(cv::Mat& frame, int clsId, float conf, int x1, int y1, int x2, int y2, float depth);
    
    /**
     * @brief Get class name for a class ID
     * @param clsId Class ID
     * @return Class name
     */
    std::string getClassName(int clsId);
    
    /**
     * @brief Publish annotated RGB image
     * @param frame Frame to publish
     */
    void publishAnnotatedImage(const cv::Mat& frame);
    
    /**
     * @brief Publish bounding boxes with depth information
     * @param bboxDepthData Bounding box data with depth
     */
    void publishBboxDepth(const std::vector<float>& bboxDepthData);
    
    // ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // ROS parameters
    std::string image_topic_;
    std::string depth_image_topic_;
    std::string bbox_topic_;
    std::string annotation_topic_;
    std::string bbox_depth_topic_;
    
    // ROS subscribers
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber bbox_sub_;
    
    // ROS publishers
    ros::Publisher image_pub_;
    ros::Publisher bbox_depth_pub_;
    
    // ROS timer
    ros::Timer process_timer_;
    
    // Latest messages
    sensor_msgs::ImageConstPtr latest_image_;
    sensor_msgs::ImageConstPtr latest_depth_;
    std_msgs::Float32MultiArrayConstPtr latest_bbox_;
    std_msgs::Header latest_image_header_;
    
    
    // Flag to track if we have a valid header
    bool has_valid_header_;
};

#endif // DISTANCE_NODE_HPP