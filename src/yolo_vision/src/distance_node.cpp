#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>
#include <string>

/**
 * Node for calculating distances from depth images using bounding boxes from YOLO.
 * Subscribes to YOLO bounding boxes and depth images.
 * Publishes annotated RGB images and bounding boxes with depth information.
 */
class DistanceNode {
public:
    DistanceNode() : nh_("~") {
        // Get parameters
        nh_.param<std::string>("image_topic", image_topic_, "/camera/rgb/image_raw");
        nh_.param<std::string>("depth_image_topic", depth_image_topic_, "/camera/depth/image_raw");
        nh_.param<std::string>("bbox_topic", bbox_topic_, "/yolo/bboxes");
        nh_.param<std::string>("annotated_image_topic", annotation_topic_, "/distance/annotated_image");
        nh_.param<std::string>("bbox_depth_topic", bbox_depth_topic_, "/distance/bbox_depth");

        // Set up class name mapping
        class_names_[0] = "grasshopper";
        class_names_[1] = "box";

        // Setup ROS communication
        setupRosComm();
    }

    void run() {
        // Use a timer to process data at fixed rate (10 Hz)
        process_timer_ = nh_.createTimer(ros::Duration(0.1), &DistanceNode::processData, this);
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    
    // ROS topic names
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
    
    // Timer for processing at fixed rate
    ros::Timer process_timer_;
    
    // Latest messages
    sensor_msgs::ImageConstPtr latest_image_;
    sensor_msgs::ImageConstPtr latest_depth_;
    std_msgs::Float32MultiArrayConstPtr latest_bbox_;
    
    // Class name mapping
    std::unordered_map<int, std::string> class_names_;

    /**
     * Set up ROS subscribers and publishers
     */
    void setupRosComm() {
        // Subscribe to RGB images
        image_sub_ = nh_.subscribe(image_topic_, 1, &DistanceNode::imageCallback, this);
        
        // Subscribe to depth images
        depth_sub_ = nh_.subscribe(depth_image_topic_, 1, &DistanceNode::depthCallback, this);
        
        // Subscribe to bounding boxes
        bbox_sub_ = nh_.subscribe(bbox_topic_, 1, &DistanceNode::bboxCallback, this);
        
        // Publishers
        image_pub_ = nh_.advertise<sensor_msgs::Image>(annotation_topic_, 1);
        bbox_depth_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(bbox_depth_topic_, 1);
    }

    /**
     * Callback for RGB images
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_image_ = msg;
    }

    /**
     * Callback for depth images
     */
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_depth_ = msg;
    }

    /**
     * Callback for bounding box data
     */
    void bboxCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
        latest_bbox_ = msg;
    }

    /**
     * Process all available data (called by timer)
     */
    void processData(const ros::TimerEvent& event) {
        // Check if we have all required data
        if (!latest_image_ || !latest_depth_ || !latest_bbox_) {
            return;
        }
        
        try {
            // Convert ROS Image messages to OpenCV format
            cv_bridge::CvImagePtr rgb_cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb_frame = rgb_cv_ptr->image;
            
            cv::Mat depth_image = convertDepthImageToCv2(latest_depth_);
            
            // Process bounding boxes and calculate distances
            std::vector<float> bbox_depth_data = processBboxesWithDepth(rgb_frame, depth_image, latest_bbox_->data);
            
            // Publish the annotated image
            publishAnnotatedImage(rgb_frame);
            
            // Publish bounding boxes with depth
            publishBboxDepth(bbox_depth_data);
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("CV Bridge error: " << e.what());
        } catch (std::exception& e) {
            ROS_ERROR_STREAM("Error in processing data: " << e.what());
        }
    }

    /**
     * Convert ROS depth image to meters in OpenCV format
     */
    cv::Mat convertDepthImageToCv2(const sensor_msgs::ImageConstPtr& ros_image) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth;
            cv_ptr->image.convertTo(depth, CV_32F, 0.001); // Convert from mm to meters
            return depth;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("Error converting depth image: " << e.what());
            return cv::Mat(480, 640, CV_32F, cv::Scalar(0));
        }
    }

    /**
     * Calculate the average depth for a bounding box
     */
    float processBboxDepth(const cv::Mat& depth, const cv::Rect& bbox) {
        // Safety checks
        if (depth.empty()) {
            return 0.0;
        }
        
        // Ensure coordinates are within image bounds
        cv::Rect safe_bbox = bbox & cv::Rect(0, 0, depth.cols, depth.rows);
        
        if (safe_bbox.width <= 0 || safe_bbox.height <= 0) {
            return 0.0;
        }
        
        // Extract depth values within bounding box
        cv::Mat roi_depth = depth(safe_bbox);
        
        // Calculate average of non-zero depth values
        cv::Mat mask = roi_depth > 0.1; // Filter out very small values
        if (cv::countNonZero(mask) > 0) {
            cv::Scalar mean = cv::mean(roi_depth, mask);
            return static_cast<float>(mean[0]);
        } else {
            return 0.0;
        }
    }

    /**
     * Process bounding boxes, calculate depths, and annotate the image
     */
    std::vector<float> processBboxesWithDepth(cv::Mat& frame, const cv::Mat& depth_image, 
                                             const std::vector<float>& bbox_data) {
        // Copy bbox data for manipulation
        std::vector<float> bbox_depth_data;
        int num_boxes = static_cast<int>(bbox_data[0]);
        
        // First element is number of boxes
        bbox_depth_data.push_back(num_boxes);
        
        if (num_boxes == 0) {
            return bbox_depth_data;
        }
        
        // Calculate bbox data length per detection
        // [cls_id, conf, x1, y1, x2, y2]
        const int bbox_stride = 6;
        
        for (int i = 0; i < num_boxes; i++) {
            // Extract bbox data for this detection
            int start_idx = 1 + i * bbox_stride;
            int cls_id = static_cast<int>(bbox_data[start_idx]);
            float conf = bbox_data[start_idx + 1];
            int x1 = static_cast<int>(bbox_data[start_idx + 2]);
            int y1 = static_cast<int>(bbox_data[start_idx + 3]);
            int x2 = static_cast<int>(bbox_data[start_idx + 4]);
            int y2 = static_cast<int>(bbox_data[start_idx + 5]);
            
            // Calculate depth
            cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);
            float avg_depth = processBboxDepth(depth_image, bbox);
            
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

    /**
     * Get class name for a class ID
     */
    std::string getClassName(int cls_id) {
        auto it = class_names_.find(cls_id);
        if (it != class_names_.end()) {
            return it->second;
        } else {
            return "class_" + std::to_string(cls_id);
        }
    }

    /**
     * Annotate a frame with bounding box, class, confidence, and depth
     */
    void annotateFrame(cv::Mat& frame, int cls_id, float conf, int x1, int y1, int x2, int y2, float depth) {
        // Draw rectangle
        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        
        // Add class name and confidence
        std::string cls_name = getClassName(cls_id);
        std::string label = cls_name + " " + std::to_string(conf).substr(0, 4);
        
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
            char buffer[50];
            std::sprintf(buffer, "Distance: %.2fm", depth);
            depth_text = buffer;
        }
        
        cv::Size depth_text_size = cv::getTextSize(depth_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
        
        // Position depth text below the class label
        int depth_y = text_y + depth_text_size.height + 5;
        cv::putText(frame, depth_text, cv::Point(x1, depth_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    /**
     * Publish the annotated RGB image
     */
    void publishAnnotatedImage(const cv::Mat& frame) {
        if (!latest_image_) {
            return;
        }
        
        try {
            cv_bridge::CvImage annotated_img;
            annotated_img.header = latest_image_->header;
            annotated_img.encoding = sensor_msgs::image_encodings::BGR8;
            annotated_img.image = frame;
            
            image_pub_.publish(annotated_img.toImageMsg());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("Error publishing annotated image: " << e.what());
        }
    }

    /**
     * Publish bounding boxes with depth information
     */
    void publishBboxDepth(const std::vector<float>& bbox_depth_data) {
        if (!latest_image_) {
            return;
        }
        
        try {
            std_msgs::Float32MultiArray msg;
            
            // Set dimensions
            msg.layout.dim.resize(1);
            msg.layout.dim[0].label = "bbox_depth";
            msg.layout.dim[0].size = bbox_depth_data.size();
            msg.layout.dim[0].stride = bbox_depth_data.size();
            
            // Set data
            msg.data = bbox_depth_data;
            
            // Publish
            bbox_depth_pub_.publish(msg);
        } catch (std::exception& e) {
            ROS_ERROR_STREAM("Error publishing bbox depth data: " << e.what());
        }
    }
};

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "distance_node");
        DistanceNode node;
        node.run();
    } catch (ros::Exception& e) {
        ROS_ERROR_STREAM("ROS error: " << e.what());
        return 1;
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Error: " << e.what());
        return 1;
    }
    
    return 0;
}