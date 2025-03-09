#include "YoloNode.hpp"
#include <fstream>

YoloNode::YoloNode(ros::NodeHandle& nh) :
    nh_(nh),
    it_(nh),
    conf_threshold_(0.5),
    nms_threshold_(0.4),
    max_detections_(100),
    min_process_interval_(ros::Duration(1.0 / 30.0)) // 30 FPS
{
    // Load Parameters
    std::string model_path, image_topic, annotation_topic;

    nh_.param<std::string>("model_path", model_path);
    nh_.param<std::string>("image_topic", image_topic);
    nh_.param<std::string>("annotated_image_topic", annotation_topic);
    
    // Optional parameters with defaults
    nh_.param<float>("conf_threshold", conf_threshold_, 0.5);
    nh_.param<float>("nms_threshold", nms_threshold_, 0.4);
    nh_.param<int>("max_detections", max_detections_, 100);
    
    double process_rate;
    nh_.param<double>("process_rate", process_rate, 30.0);
    min_process_interval_ = ros::Duration(1.0 / process_rate);

    // Load Yolo model
    loadModel(model_path);

    // Set up Subscribers and Publishers
    image_sub_ = it_.subscribe(image_topic, 1, &YoloNode::imageCallback, this);
    image_pub_ = it_.advertise(annotation_topic, 1);

    // Initialize last process time
    last_process_time_ = ros::Time::now();

    ROS_INFO("YOLO node initialized");
}

void YoloNode::loadModel(const std::string& model_path)
{
    try {
        // Load YOLO model
        model_ = cv::dnn::readNet(model_path);
        
        // Check if GPU is available and set preference
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            model_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            model_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            ROS_INFO("CUDA enabled.");
        } else {
            model_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            model_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            ROS_INFO("CUDA not available. Running on CPU.");
        }
        
        // Load class names if available (from a .txt file with same name as model)
        std::string class_file = model_path.substr(0, model_path.find_last_of(".")) + ".txt";
        std::ifstream ifs(class_file.c_str());
        if (ifs.is_open()) {
            std::string line;
            while (std::getline(ifs, line)) {
                class_names_.push_back(line);
            }
            ROS_INFO("Loaded %zu class names", class_names_.size());
        } else {
            ROS_WARN("Class names file not found: %s", class_file.c_str());
        }
    } catch (const cv::Exception& e) {
        ROS_ERROR("Error loading YOLO model: %s", e.what());
        throw;
    }
}

cv::Mat YoloNode::preprocess(const cv::Mat& frame) 
{
    // Pre-process image for YOLO
    cv::Mat blob = cv::dnn::blobFromImage(
        frame,          // Input image
        1/255.0,        // Scale factor
        cv::Size(640, 640), // Target size
        cv::Scalar(0,0,0),  // Mean subtraction
        true,           // Swap Red and Blue channels
        false           // Do not crop
    );
    
    return blob;
}

std::vector<Detection> YoloNode::runInference(const cv::Mat& blob, const cv::Mat& frame) 
{
    // Set the input to the model
    model_.setInput(blob);
    
    // Run forward pass
    std::vector<cv::Mat> outputs;
    model_.forward(outputs, model_.getUnconnectedOutLayersNames());
    
    // Post-process results
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    // Process detections based on YOLO model format
    for (size_t i = 0; i < outputs.size(); ++i) {
        // Get detection results
        float* data = (float*)outputs[i].data;
        
        for (int j = 0; j < outputs[i].rows; ++j) {
            // Get confidence scores
            float confidence = data[j * outputs[i].cols + 4];
            
            // Filter by confidence threshold
            if (confidence >= conf_threshold_) {
                // Get class scores starting from index 5
                cv::Mat scores = outputs[i].row(j).colRange(5, outputs[i].cols);
                cv::Point classId;
                double maxScore;
                
                // Get the max score and its index
                cv::minMaxLoc(scores, nullptr, &maxScore, nullptr, &classId);
                
                if (maxScore > conf_threshold_) {
                    // Calculate bounding box coordinates
                    float x = data[j * outputs[i].cols + 0];
                    float y = data[j * outputs[i].cols + 1];
                    float w = data[j * outputs[i].cols + 2];
                    float h = data[j * outputs[i].cols + 3];
                    
                    // Convert to pixel coordinates
                    int left = static_cast<int>((x - w/2) * frame.cols);
                    int top = static_cast<int>((y - h/2) * frame.rows);
                    int width = static_cast<int>(w * frame.cols);
                    int height = static_cast<int>(h * frame.rows);
                    
                    // Add to our collections
                    classIds.push_back(classId.x);
                    confidences.push_back(static_cast<float>(maxScore));
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }
    }
    
    // Apply Non-Maximum Suppression to remove overlapping boxes
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
    
    // Store final detections
    std::vector<Detection> detections;
    
    // Limit to max_detections 
    int num_to_process = std::min(static_cast<int>(indices.size()), max_detections_);
    
    // Process detections
    for (int i = 0; i < num_to_process; ++i) {
        int idx = indices[i];
        
        Detection det;
        det.bbox = boxes[idx];
        det.class_id = classIds[idx];
        det.confidence = confidences[idx];
        
        detections.push_back(det);
    }
    
    return detections;
}

void YoloNode::visualizeDetections(cv::Mat& frame, const std::vector<Detection>& detections) 
{
    for (const auto& det : detections) {
        // Draw bounding box (green rectangle)
        cv::rectangle(frame, det.bbox, cv::Scalar(0, 255, 0), 2);
        
        // Create label with class name and confidence
        std::string className = (det.class_id < class_names_.size()) ? 
            class_names_[det.class_id] : std::to_string(det.class_id);
        std::string label = className + " " + cv::format("%.2f", det.confidence);
        
        // Get text size for positioning
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseLine);
        
        // Ensure label is within image bounds
        int text_y = std::max(det.bbox.y, labelSize.height + 5);
        
        // Draw text
        cv::putText(frame, label, cv::Point(det.bbox.x, text_y), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
}

void YoloNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Rate limiting
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_process_time_) < min_process_interval_) {
        return;
    }
    
    try {
        // Convert ROS Image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        // 1. Preprocess the image
        cv::Mat blob = preprocess(frame);
        
        // 2. Run inference
        std::vector<Detection> detections = runInference(blob, frame);
        
        // 3. Visualize detections
        visualizeDetections(frame, detections);
        
        // 4. Convert back to ROS message and publish
        cv_ptr->header = msg->header;  // Preserve original header
        image_pub_.publish(cv_ptr->toImageMsg());
        
        // Update process time
        last_process_time_ = current_time;
        
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Standard exception: %s", e.what());
    }
}