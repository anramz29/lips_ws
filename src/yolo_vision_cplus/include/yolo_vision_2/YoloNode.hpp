#ifndef YOLO_NODE_HPP
#define YOLO_NODE_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <string>
#include <vector>

// Structure to hold detection results
struct Detection {
    cv::Rect bbox;       // Bounding box
    int class_id;        // Class ID 
    float confidence;    // Confidence score
};

class YoloNode
{
private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Image transport for efficient image handling
    image_transport::ImageTransport it_;
    
    // Subscribers and publishers
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    // YOLO model
    cv::dnn::Net model_;
    float conf_threshold_;
    float nms_threshold_;
    int max_detections_;
    std::vector<std::string> class_names_;
    
    // Rate limiting
    ros::Time last_process_time_;
    ros::Duration min_process_interval_;
    
    // Main callback function for image processing
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // Helper function to load YOLO model
    void loadModel(const std::string& model_path);
    
    // Helper for preprocessing image before inference
    cv::Mat preprocess(const cv::Mat& frame);
    
    // Run YOLO inference on preprocessed image
    std::vector<Detection> runInference(const cv::Mat& blob, const cv::Mat& frame);
    
    // Draw detections on the image
    void visualizeDetections(cv::Mat& frame, const std::vector<Detection>& detections);

public:
    // Constructor with node handle parameter
    YoloNode(ros::NodeHandle& nh);
    
    // Destructor
    ~YoloNode() {}
    
    // Function to start node operation
    void spin() {
        ros::spin();
    }
};

#endif // YOLO_NODE_HPP