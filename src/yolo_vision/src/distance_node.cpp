#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

class DistanceNode
{
public:
    DistanceNode() : private_nh_("~")
    {
        // Load parameters from parameter server
        private_nh_.param<std::string>("annotated_image_topic", yolo_image_topic_, "annotated_image");
        private_nh_.param<std::string>("depth_image_topic", depth_image_topic_, "depth_image");
        private_nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "bbox_depth");
        private_nh_.param<std::string>("visualization_topic", visualization_topic_, "visualization");
        
        // Publishers
        viz_pub_ = nh_.advertise<sensor_msgs::Image>(visualization_topic_, 10);
        bbox_depth_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(bbox_depth_topic_, 10);
        
        // Subscribers
        yolo_sub_ = nh_.subscribe(yolo_image_topic_, 10, &DistanceNode::yoloCallback, this);
        depth_sub_ = nh_.subscribe(depth_image_topic_, 10, &DistanceNode::depthCallback, this);
    }

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers
    ros::Publisher viz_pub_;
    ros::Publisher bbox_depth_pub_;
    
    // Subscribers
    ros::Subscriber yolo_sub_;
    ros::Subscriber depth_sub_;
    
    // ROS parameters
    std::string yolo_image_topic_;
    std::string depth_image_topic_;
    std::string bbox_depth_topic_;
    std::string visualization_topic_;
    
    // Cache for the latest messages
    sensor_msgs::ImageConstPtr last_yolo_msg_;
    sensor_msgs::ImageConstPtr last_depth_msg_;

    void yoloCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            last_yolo_msg_ = msg;
            processImages();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in YOLO callback: " << e.what());
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            last_depth_msg_ = msg;
            processImages();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in depth callback: " << e.what());
        }
    }

    cv::Rect getBBoxFromMask(const cv::Mat& img)
    {
        // Create mask for green pixels
        cv::Mat mask;
        cv::inRange(img, cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 0), mask);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            return cv::Rect(0, 0, 0, 0); // Return empty rect if no contours
        }
        
        // Find the largest contour
        auto largest_contour = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        
        return cv::boundingRect(*largest_contour);
    }

    void processImages()
    {
        if (!last_yolo_msg_ || !last_depth_msg_) {
            return;
        }

        try {
            // Convert ROS image messages to OpenCV images
            cv_bridge::CvImagePtr cv_yolo = cv_bridge::toCvCopy(last_yolo_msg_, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_yolo->image;
            
            cv_bridge::CvImagePtr cv_depth;
            cv::Mat depth;
            
            if (last_depth_msg_->encoding == "32FC1") {
                cv_depth = cv_bridge::toCvCopy(last_depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
                depth = cv_depth->image;
                if (cv::mean(depth)[0] > 100) {
                    depth = depth / 1000.0;
                }
            } else {
                cv_depth = cv_bridge::toCvCopy(last_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
                depth = cv_depth->image;
                depth.convertTo(depth, CV_32F, 0.001);
            }
            
            // Get the bounding box from the YOLO annotation
            cv::Rect bbox = getBBoxFromMask(img);
            
            if (bbox.width > 0 && bbox.height > 0) {
                // Extract ROI from depth image
                cv::Mat roi_depth = depth(bbox);
                
                // Calculate mean depth (ignoring zeros)
                cv::Mat valid_depths;
                cv::findNonZero(roi_depth, valid_depths);
                
                if (valid_depths.total() > 0) {
                    float avg_depth = 0.0;
                    int count = 0;
                    
                    for (int i = 0; i < roi_depth.rows; i++) {
                        for (int j = 0; j < roi_depth.cols; j++) {
                            float d = roi_depth.at<float>(i, j);
                            if (d > 0) {
                                avg_depth += d;
                                count++;
                            }
                        }
                    }
                    
                    if (count > 0) {
                        avg_depth /= count;
                        
                        // Create message with bbox coordinates and depth
                        std_msgs::Float32MultiArray msg;
                        msg.data.push_back(static_cast<float>(bbox.x));
                        msg.data.push_back(static_cast<float>(bbox.y));
                        msg.data.push_back(static_cast<float>(bbox.width));
                        msg.data.push_back(static_cast<float>(bbox.height));
                        msg.data.push_back(avg_depth);
                        bbox_depth_pub_.publish(msg);
                        
                        // Add depth text to the image
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << avg_depth << "m";
                        std::string depth_text = ss.str();
                        
                        int baseline = 0;
                        cv::Size text_size = cv::getTextSize(depth_text, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
                        int text_x = bbox.x + (bbox.width - text_size.width) / 2;
                        int text_y = (bbox.y > 30) ? bbox.y - 10 : bbox.y + 30;
                        
                        cv::putText(img, depth_text, cv::Point(text_x, text_y),
                                  cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            
            // Publish visualization
            sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(last_yolo_msg_->header, "bgr8", img).toImageMsg();
            viz_pub_.publish(viz_msg);
            
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in processImages: " << e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_node");
    DistanceNode node;
    ros::spin();
    return 0;
}