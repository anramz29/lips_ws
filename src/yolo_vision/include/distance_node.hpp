#ifndef DISTANCE_NODE_HPP
#define DISTANCE_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>

class DistanceNode {
public:
    DistanceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DistanceNode() = default;

private:
    // ROS members
    ros::NodeHandle nh_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher viz_pub_;
    ros::Publisher bbox_depth_pub_;
    
    // Message storage
    sensor_msgs::ImageConstPtr last_yolo_msg_;
    sensor_msgs::ImageConstPtr last_depth_msg_;
    
    // OpenCV bridge
    cv_bridge::CvImagePtr cv_bridge_ptr_;
    
    // Callback functions
    void yoloCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void processImages();
    
    // Helper functions
    cv::Rect getBboxFromMask(const cv::Mat& img);
};

#endif // DISTANCE_NODE_HPP