#ifndef YOLO_NODE_HPP
#define YOLO_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class YoloNode {
public:
    YoloNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~YoloNode() = default;

private:
    // ROS members
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    
    // OpenCV members
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    float conf_threshold_;
    float nms_threshold_;
    int max_detections_;
    
    // Processing control
    ros::Time last_process_time_;
    ros::Duration min_process_interval_;
    
    // Callback
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // Helper functions
    void loadModel(const std::string& model_path, const std::string& config_path);
    void loadClassNames(const std::string& names_path);
    cv::Mat detectObjects(const cv::Mat& frame);
    void drawDetections(cv::Mat& frame, const std::vector<cv::Mat>& detections);
};

#endif // YOLO_NODE_HPP