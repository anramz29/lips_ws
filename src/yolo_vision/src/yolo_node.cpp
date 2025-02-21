#include "yolo_node.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

YoloNode::YoloNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    // Load parameters
    std::string model_path, config_path, names_path, image_topic, annotation_topic;
    pnh.param<std::string>("model_path", model_path, "");
    pnh.param<std::string>("config_path", config_path, "");
    pnh.param<std::string>("names_path", names_path, "");
    pnh.param<std::string>("image_topic", image_topic, "/camera/image_raw");
    pnh.param<std::string>("annotated_image_topic", annotation_topic, "/yolo/annotated_image");
    
    // Load detection parameters
    pnh.param<float>("conf_threshold", conf_threshold_, 0.5);
    pnh.param<float>("nms_threshold", nms_threshold_, 0.45);
    pnh.param<int>("max_detections", max_detections_, 1);
    
    // Initialize processing control
    min_process_interval_ = ros::Duration(0.1); // 10 Hz maximum
    last_process_time_ = ros::Time::now();
    
    // Load YOLO model and class names
    loadModel(model_path, config_path);
    loadClassNames(names_path);
    
    // Setup ROS communication
    image_sub_ = nh_.subscribe(image_topic, 1, &YoloNode::imageCallback, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>(annotation_topic, 1);
    
    // Try to use CUDA if available
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    
    ROS_INFO("YoloNode initialized.");
}

void YoloNode::loadModel(const std::string& model_path, const std::string& config_path) {
    try {
        net_ = cv::dnn::readNetFromDarknet(config_path, model_path);
        ROS_INFO("YOLO model loaded successfully.");
    } catch (const cv::Exception& e) {
        ROS_ERROR("Failed to load YOLO model: %s", e.what());
        ros::shutdown();
    }
}

void YoloNode::loadClassNames(const std::string& names_path) {
    std::ifstream file(names_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open class names file: %s", names_path.c_str());
        return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            class_names_.push_back(line);
        }
    }
}

void YoloNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Rate limiting
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_process_time_) < min_process_interval_) {
        return;
    }
    
    try {
        // Convert ROS Image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        // Prepare image for network
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);
        
        // Get output layer names
        std::vector<std::string> output_names = net_.getUnconnectedOutLayersNames();
        std::vector<cv::Mat> detections;
        net_.forward(detections, output_names);
        
        // Process detections and draw on frame
        drawDetections(frame, detections);
        
        // Convert back to ROS message and publish
        cv_bridge::CvImage annotated_msg;
        annotated_msg.header = msg->header;
        annotated_msg.encoding = sensor_msgs::image_encodings::BGR8;
        annotated_msg.image = frame;
        
        image_pub_.publish(annotated_msg.toImageMsg());
        last_process_time_ = current_time;
        
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV error: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Error in image callback: %s", e.what());
    }
}

void YoloNode::drawDetections(cv::Mat& frame, const std::vector<cv::Mat>& detections) {
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    
    // Process detections
    for (const auto& output : detections) {
        for (int i = 0; i < output.rows; ++i) {
            cv::Mat scores = output.row(i).colRange(5, output.cols);
            cv::Point class_id_point;
            double confidence;
            cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &class_id_point);
            
            if (confidence > conf_threshold_) {
                float cx = output.at<float>(i, 0);
                float cy = output.at<float>(i, 1);
                float w = output.at<float>(i, 2);
                float h = output.at<float>(i, 3);
                
                int left = static_cast<int>((cx - w/2) * frame.cols);
                int top = static_cast<int>((cy - h/2) * frame.rows);
                int width = static_cast<int>(w * frame.cols);
                int height = static_cast<int>(h * frame.rows);
                
                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(static_cast<float>(confidence));
                class_ids.push_back(class_id_point.x);
            }
        }
    }
    
    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
    
    // Draw detections (limited to max_detections_)
    for (size_t i = 0; i < std::min(static_cast<size_t>(max_detections_), indices.size()); ++i) {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int class_id = class_ids[idx];
        float conf = confidences[idx];
        
        // Draw rectangle
        cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
        
        // Draw label
        std::string label = (class_id < class_names_.size() ? class_names_[class_id] : std::to_string(class_id)) +
                           " " + std::to_string(conf).substr(0, 4);
        
        int baseline;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
        int label_y = std::max(box.y, label_size.height + 5);
        
        cv::putText(frame, label, cv::Point(box.x, label_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    YoloNode node(nh, pnh);
    ros::spin();
    
    return 0;
}