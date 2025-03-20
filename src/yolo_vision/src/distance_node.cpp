#include "yolo_vision/distance_node.hpp"

DistanceNode::DistanceNode() : private_nh_("~"), has_valid_header_(false) {
    // Get parameters
    private_nh_.param<std::string>("image_topic", image_topic_, "/camera/rgb/image_raw");
    private_nh_.param<std::string>("depth_image_topic", depth_image_topic_, "/camera/depth/image_raw");
    private_nh_.param<std::string>("bbox_topic", bbox_topic_, "/yolo/detections");
    private_nh_.param<std::string>("annotated_image_topic", annotation_topic_, "/distance_node/annotated_image");
    private_nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "/distance_node/bbox_depth");
    
    // Initialize ROS communication
    setupRosCommunication();
}

void DistanceNode::setupRosCommunication() {
    // Subscribe to RGB images
    image_sub_ = nh_.subscribe(image_topic_, 1, &DistanceNode::imageCallback, this);
    
    // Subscribe to depth images
    depth_sub_ = nh_.subscribe(depth_image_topic_, 1, &DistanceNode::depthCallback, this);
    
    // Subscribe to bounding boxes from YOLO
    bbox_sub_ = nh_.subscribe(bbox_topic_, 1, &DistanceNode::bboxCallback, this);
    
    // Publishers
    image_pub_ = nh_.advertise<sensor_msgs::Image>(annotation_topic_, 1);
    bbox_depth_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(bbox_depth_topic_, 1);
    
    // Timer for processing at a fixed rate (10 Hz)
    process_timer_ = nh_.createTimer(ros::Duration(0.1), &DistanceNode::processData, this);
}

void DistanceNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    latest_image_ = msg;
    latest_image_header_ = msg->header;
    has_valid_header_ = true;
}

void DistanceNode::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    latest_depth_ = msg;
}

void DistanceNode::bboxCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
    latest_bbox_ = msg;
}

void DistanceNode::processData(const ros::TimerEvent& event) {
    // Check if we have all required data
    if (!latest_image_ || !latest_depth_ || !latest_bbox_) {
        return;
    }
    
    try {
        // Convert ROS Image messages to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_frame = cv_ptr->image;
        cv::Mat depth_image = convertDepthImageToCv2(latest_depth_);
        
        // Process bounding boxes and calculate distances
        std::vector<float> bbox_data(latest_bbox_->data.begin(), latest_bbox_->data.end());
        std::vector<float> bbox_depth_data = processBboxesWithDepth(rgb_frame, depth_image, bbox_data);
        
        // Publish the annotated image
        publishAnnotatedImage(rgb_frame);
        
        // Publish bounding boxes with depth
        publishBboxDepth(bbox_depth_data);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
    }
    catch (std::exception& e) {
        ROS_ERROR("Error in processing data: %s", e.what());
    }
}

cv::Mat DistanceNode::convertDepthImageToCv2(const sensor_msgs::ImageConstPtr& rosImage) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(rosImage, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;
        
        // Convert from mm to meters
        cv::Mat depth_meters;
        depth_image.convertTo(depth_meters, CV_32F, 0.001);
        return depth_meters;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error converting depth image: %s", e.what());
        return cv::Mat(480, 640, CV_32F, cv::Scalar(0));
    }
}

float DistanceNode::processBboxDepth(const cv::Mat& depth, const cv::Rect& bbox) {
    // Safety checks
    if (depth.empty()) {
        return 0.0f;
    }
    
    // Ensure coordinates are within image bounds
    cv::Rect safe_bbox = bbox & cv::Rect(0, 0, depth.cols, depth.rows);
    
    if (safe_bbox.width <= 0 || safe_bbox.height <= 0) {
        return 0.0f;
    }
    
    // Extract depth values within bounding box
    cv::Mat roi_depth = depth(safe_bbox);
    
    // Create mask for valid depth values (> 0.1m)
    cv::Mat mask = roi_depth > 0.1f;
    
    // If no valid depths, return 0
    if (cv::countNonZero(mask) == 0) {
        return 0.0f;
    }
    
    // Calculate mean of non-zero depth values
    cv::Scalar mean_val = cv::mean(roi_depth, mask);
    return static_cast<float>(mean_val[0]);
}

std::vector<float> DistanceNode::processBboxesWithDepth(cv::Mat& frame, const cv::Mat& depthImage, 
                                                      const std::vector<float>& bboxData) {
    if (bboxData.empty()) {
        return bboxData;
    }
    
    // Get number of boxes
    int num_boxes = static_cast<int>(bboxData[0]);
    
    if (num_boxes == 0) {
        return bboxData;
    }
    
    // Calculate bbox data length per detection
    // [cls_id, conf, x1, y1, x2, y2]
    const int bbox_stride = 6;
    
    // Create new bbox data with depth
    std::vector<float> bbox_depth_data;
    bbox_depth_data.push_back(num_boxes);  // Start with number of boxes
    
    for (int i = 0; i < num_boxes; ++i) {
        // Extract bbox data for this detection
        int start_idx = 1 + i * bbox_stride;
        int cls_id = static_cast<int>(bboxData[start_idx]);
        float conf = bboxData[start_idx + 1];
        int x1 = static_cast<int>(bboxData[start_idx + 2]);
        int y1 = static_cast<int>(bboxData[start_idx + 3]);
        int x2 = static_cast<int>(bboxData[start_idx + 4]);
        int y2 = static_cast<int>(bboxData[start_idx + 5]);
        
        // Calculate depth
        cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);
        float avg_depth = processBboxDepth(depthImage, bbox);
        
        // Add to new data with depth
        // [cls_id, conf, x1, y1, x2, y2, depth]
        bbox_depth_data.push_back(cls_id);
        bbox_depth_data.push_back(conf);
        bbox_depth_data.push_back(x1);
        bbox_depth_data.push_back(y1);
        bbox_depth_data.push_back(x2);
        bbox_depth_data.push_back(y2);
        bbox_depth_data.push_back(avg_depth);
        
        // Annotate the frame
        annotateFrame(frame, cls_id, conf, x1, y1, x2, y2, avg_depth);
    }
    
    return bbox_depth_data;
}

void DistanceNode::annotateFrame(cv::Mat& frame, int clsId, float conf, int x1, int y1, int x2, int y2, float depth) {
    // Draw rectangle
    cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
    
    // Add class name and confidence
    std::string cls_name = getClassName(clsId);
    std::stringstream ss;
    ss << cls_name << " " << std::fixed << std::setprecision(2) << conf;
    std::string label = ss.str();
    
    // Draw class and confidence
    int baseline = 0;
    cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
    int text_y = std::max(y1, label_size.height + 5);
    cv::putText(frame, label, cv::Point(x1, text_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    
    // Draw depth information
    std::string depth_text;
    if (depth <= 0) {
        depth_text = "Distance: Unknown";
    } else {
        std::stringstream ss_depth;
        ss_depth << "Distance: " << std::fixed << std::setprecision(2) << depth << "m";
        depth_text = ss_depth.str();
    }
    
    cv::Size depth_text_size = cv::getTextSize(depth_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
    
    // Position depth text below the class label
    int depth_y = text_y + depth_text_size.height + 5;
    cv::putText(frame, depth_text, cv::Point(x1, depth_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

std::string DistanceNode::getClassName(int clsId) {
    // Placeholder - you should use a proper mapping
    std::map<int, std::string> class_names = {
        {0, "grasshopper"},
        {1, "box"}
    };
    
    auto it = class_names.find(clsId);
    if (it != class_names.end()) {
        return it->second;
    } else {
        return "class_" + std::to_string(clsId);
    }
}

void DistanceNode::publishAnnotatedImage(const cv::Mat& frame) {
    if (!has_valid_header_) {
        return;
    }
    
    try {
        sensor_msgs::ImagePtr annotated_msg = cv_bridge::CvImage(latest_image_header_, 
                                                                "bgr8", 
                                                                frame).toImageMsg();
        image_pub_.publish(annotated_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error publishing annotated image: %s", e.what());
    }
}

void DistanceNode::publishBboxDepth(const std::vector<float>& bboxDepthData) {
    if (!has_valid_header_) {
        return;
    }
    
    try {
        // Create message
        std_msgs::Float32MultiArray msg;
        
        // Set dimensions
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].label = "bbox_depth";
        msg.layout.dim[0].size = bboxDepthData.size();
        msg.layout.dim[0].stride = bboxDepthData.size();
        
        // Set data
        msg.data = bboxDepthData;
        
        // Publish
        bbox_depth_pub_.publish(msg);
    } catch (std::exception& e) {
        ROS_ERROR("Error publishing bbox depth data: %s", e.what());
    }
}

void DistanceNode::spin() {
    // Start processing in ROS event loop
    ros::spin();
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "distance_node");
    
    // Create node instance
    DistanceNode node;
    
    // Run the node
    node.spin();
    
    return 0;
}