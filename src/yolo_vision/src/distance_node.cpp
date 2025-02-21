#include "distance_node.hpp"

DistanceNode::DistanceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    // Load parameters
    std::string yolo_image_topic, depth_image_topic, bbox_depth_topic, visualization_topic;
    pnh.param<std::string>("annotated_image_topic", yolo_image_topic, "/yolo/annotated_image");
    pnh.param<std::string>("depth_image_topic", depth_image_topic, "/camera/depth/image_raw");
    pnh.param<std::string>("bbox_depth_topic", bbox_depth_topic, "/camera/bbox_depth");
    pnh.param<std::string>("visualization_topic", visualization_topic, "/camera/distance_viz");
    
    // Initialize publishers
    viz_pub_ = nh_.advertise<sensor_msgs::Image>(visualization_topic, 10);
    bbox_depth_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(bbox_depth_topic, 10);
    
    // Initialize subscribers
    yolo_sub_ = nh_.subscribe(yolo_image_topic, 1, &DistanceNode::yoloCallback, this);
    depth_sub_ = nh_.subscribe(depth_image_topic, 1, &DistanceNode::depthCallback, this);
    
    ROS_INFO("Distance node initialized.");
}

void DistanceNode::yoloCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        last_yolo_msg_ = msg;
        processImages();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error in YOLO callback: " << e.what());
    }
}

void DistanceNode::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        last_depth_msg_ = msg;
        processImages();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error in depth callback: " << e.what());
    }
}

cv::Rect DistanceNode::getBboxFromMask(const cv::Mat& img) {
    // Create mask for green pixels
    cv::Mat mask;
    cv::inRange(img, cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 0), mask);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return cv::Rect();
    }
    
    // Find largest contour
    auto largest_contour = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });
    
    return cv::boundingRect(*largest_contour);
}

void DistanceNode::processImages() {
    if (!last_yolo_msg_ || !last_depth_msg_) {
        return;
    }

    try {
        // Convert YOLO image
        cv_bridge::CvImagePtr cv_yolo_ptr = cv_bridge::toCvCopy(last_yolo_msg_, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_yolo_ptr->image;

        // Convert depth image
        cv_bridge::CvImagePtr cv_depth_ptr;
        cv::Mat depth;
        
        if (last_depth_msg_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_depth_ptr = cv_bridge::toCvCopy(last_depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
            depth = cv_depth_ptr->image;
            if (cv::mean(depth)[0] > 100) {
                depth /= 1000.0;
            }
        } else {
            cv_depth_ptr = cv_bridge::toCvCopy(last_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
            depth = cv_depth_ptr->image;
            depth.convertTo(depth, CV_32F, 1.0/1000.0);
        }

        // Get bounding box
        cv::Rect bbox = getBboxFromMask(img);
        
        if (bbox.width > 0 && bbox.height > 0) {
            // Extract ROI from depth image
            cv::Mat roi_depth = depth(bbox);
            
            // Calculate average depth (ignoring zeros)
            cv::Mat valid_depths;
            cv::findNonZero(roi_depth, valid_depths);
            
            if (!valid_depths.empty()) {
                std::vector<float> depths;
                for (int i = 0; i < valid_depths.total(); i++) {
                    float d = roi_depth.at<float>(valid_depths.at<cv::Point>(i));
                    if (d > 0) {
                        depths.push_back(d);
                    }
                }
                
                if (!depths.empty()) {
                    float avg_depth = std::accumulate(depths.begin(), depths.end(), 0.0f) / depths.size();
                    
                    // Publish bbox and depth data
                    std_msgs::Float32MultiArray msg;
                    msg.data = {static_cast<float>(bbox.x), 
                              static_cast<float>(bbox.y),
                              static_cast<float>(bbox.width), 
                              static_cast<float>(bbox.height),
                              avg_depth};
                    bbox_depth_pub_.publish(msg);
                    
                    // Add depth text to image
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << avg_depth << "m";
                    std::string depth_text = ss.str();
                    
                    int baseline;
                    cv::Size text_size = cv::getTextSize(depth_text, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
                    cv::Point text_org(bbox.x + (bbox.width - text_size.width) / 2,
                                     bbox.y > 30 ? bbox.y - 10 : bbox.y + 30);
                    
                    cv::putText(img, depth_text, text_org,
                              cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
                }
            }
        }
        
        // Publish visualization
        cv_bridge::CvImage viz_msg;
        viz_msg.encoding = sensor_msgs::image_encodings::BGR8;
        viz_msg.image = img;
        viz_pub_.publish(viz_msg.toImageMsg());
        
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error in process_images: " << e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    DistanceNode node(nh, pnh);
    ros::spin();
    
    return 0;
}