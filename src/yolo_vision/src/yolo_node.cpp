#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <algorithm>
#include <numeric>



class YoloNode {
public:
    /**
     * Initialize YOLO node
     */
    YoloNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    /**
     * Destructor
     */
    ~YoloNode();

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    
    // ONNX Runtime session and environment
    std::shared_ptr<Ort::Env> ort_env_;
    std::shared_ptr<Ort::Session> ort_session_;
    
    // Model parameters
    std::string model_path_;
    float conf_threshold_;
    float iou_threshold_;
    int max_detections_;
    int image_size_;
    std::vector<std::string> class_names_;
    
    // Processing rate control
    ros::Time last_process_time_;
    ros::Duration min_process_interval_;
    
    // Image processing
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // ONNX inference functions
    cv::Mat preprocess(const cv::Mat& input_image);
    std::vector<std::vector<float>> postprocess(const std::vector<Ort::Value>& output_tensors, 
                                               const cv::Size& original_size);
    void loadClassNames(const std::string& names_file);
    cv::Mat visualizeDetections(cv::Mat& image, const std::vector<std::vector<float>>& detections);
};




namespace object_scout {

YoloNode::YoloNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) 
    : nh_(nh), 
      private_nh_(private_nh),
      conf_threshold_(0.5),
      iou_threshold_(0.45),
      max_detections_(1),
      image_size_(640)
{
    // Load parameters
    private_nh_.param<std::string>("model_path", model_path_, "");
    std::string image_topic;
    private_nh_.param<std::string>("image_topic", image_topic, "camera/image_raw");
    std::string annotation_topic;
    private_nh_.param<std::string>("annotated_image_topic", annotation_topic, "camera/yolo/annotated_image");
    
    // Additional parameters
    private_nh_.param<float>("conf_threshold", conf_threshold_, 0.5);
    private_nh_.param<float>("iou_threshold", iou_threshold_, 0.45);
    private_nh_.param<int>("max_detections", max_detections_, 1);
    private_nh_.param<int>("image_size", image_size_, 640);
    
    std::string names_file;
    private_nh_.param<std::string>("names_file", names_file, "");
    
    // Initialize ONNX Runtime
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(4);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    // Create ONNX environment
    ort_env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "yolo_node");
    
    // Check if model path is provided
    if (model_path_.empty()) {
        ROS_ERROR("Model path is empty. Please provide a valid model path.");
        ros::shutdown();
        return;
    }

    // Load the model
    try {
        ort_session_ = std::make_shared<Ort::Session>(*ort_env_, model_path_.c_str(), session_options);
        ROS_INFO("ONNX model loaded successfully from: %s", model_path_.c_str());
    } catch (const Ort::Exception& e) {
        ROS_ERROR("Failed to load ONNX model: %s", e.what());
        ros::shutdown();
        return;
    }
    
    // Load class names
    if (!names_file.empty()) {
        loadClassNames(names_file);
    }
    =
    // Initialize subscriber and publisher
    image_sub_ = nh_.subscribe(image_topic, 1, &YoloNode::imageCallback, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>(annotation_topic, 1);
    
    // Initialize rate control
    last_process_time_ = ros::Time::now();
    min_process_interval_ = ros::Duration(0.1); // 10 Hz max
    
    ROS_INFO("YOLO node initialized successfully");
}

YoloNode::~YoloNode() {
YoloNode::~YoloNode() {
    // Destructor: Cleanup is handled by shared_ptr destructors
}
void YoloNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Rate limiting
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_process_time_) < min_process_interval_) {
        return;
    }
    
    try {
        // Convert ROS Image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        // Preprocess the image
        cv::Mat blob = preprocess(frame);
        
        // Create input tensor
        std::vector<int64_t> input_dims = {1, 3, static_cast<int64_t>(image_size_), static_cast<int64_t>(image_size_)};
        size_t input_tensor_size = blob.total() * blob.elemSize();
        
        // Get input and output names
        Ort::AllocatorWithDefaultOptions allocator;
        const char* input_name = ort_session_->GetInputName(0, allocator);
        
        std::vector<const char*> input_names = {input_name};
        std::vector<const char*> output_names;
        size_t output_count = ort_session_->GetOutputCount();
        
        for (size_t i = 0; i < output_count; i++) {
            output_names.push_back(ort_session_->GetOutputName(i, allocator));
        }
        
        // Create input tensor
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            reinterpret_cast<float*>(blob.data),
            input_tensor_size / sizeof(float),
            input_dims.data(),
            input_dims.size()
        );
        
        // Run inference
        std::vector<Ort::Value> output_tensors = ort_session_->Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            &input_tensor,
            1,
            output_names.data(),
            output_names.size()
        );
        
        // Post-process detections
        std::vector<std::vector<float>> detections = postprocess(output_tensors, frame.size());
        
        // Visualize detections on the frame
        cv::Mat annotated_frame = visualizeDetections(frame, detections);
        
        // Convert back to ROS message and publish
        cv_bridge::CvImage annotated_cv_image;
        annotated_cv_image.header = msg->header;
        annotated_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        annotated_cv_image.image = annotated_frame;
        
        image_pub_.publish(annotated_cv_image.toImageMsg());
        
        // Update process time
        last_process_time_ = current_time;
    } catch (const std::exception& e) {
        ROS_ERROR("Error in image callback: %s", e.what());
    }
}

cv::Mat YoloNode::preprocess(const cv::Mat& input_image) {
    // Resize the image
    cv::Mat resized;
    cv::resize(input_image, resized, cv::Size(image_size_, image_size_), 0, 0, cv::INTER_LINEAR);
    
    // Convert to float and normalize to [0, 1]
    cv::Mat flt_image;
    resized.convertTo(flt_image, CV_32FC3, 1.0/255.0);
    
    // Create blob from image (BHWC to BCHW)
    cv::Mat blob;
    cv::dnn::blobFromImage(flt_image, blob);
    
    return blob;
}

std::vector<std::vector<float>> YoloNode::postprocess(const std::vector<Ort::Value>& output_tensors, 
                                                    const cv::Size& original_size) {
    std::vector<std::vector<float>> detections;
    
    // Get pointer to output tensor data
    const float* output_data = output_tensors[0].GetTensorData<float>();
    
    // Get dimensions of the output tensor
    Ort::TensorTypeAndShapeInfo tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_dims = tensor_info.GetShape();
    
    // For YOLOv8 format, output shape is typically [batch, num_predictions, num_classes+5]
    // where 5 represents [x, y, w, h, confidence]
    int rows = output_dims[1];  // num_predictions
    int dimensions = output_dims[2];  // num_classes + 5
    
    // Process each detection
    std::vector<std::vector<float>> temp_detections = processDetections(output_data, rows, dimensions, original_size);
    
    // Apply non-maximum suppression
    detections = applyNMS(temp_detections);
    
    return detections;
}

std::vector<std::vector<float>> YoloNode::processDetections(const float* output_data, int rows, int dimensions, const cv::Size& original_size) {
    std::vector<std::vector<float>> temp_detections;
    
    for (int i = 0; i < rows; ++i) {
        float confidence = output_data[i * dimensions + 4];
        
        // Filter by confidence threshold
        if (confidence >= conf_threshold_) {
            // Find class with highest probability
            int class_id = 0;
            float max_class_prob = 0.0;
            
            for (int j = 5; j < dimensions; ++j) {
                float class_prob = output_data[i * dimensions + j];
                if (class_prob > max_class_prob) {
                    max_class_prob = class_prob;
                    class_id = j - 5;
                }
            }
            
            // Only consider detection if class probability is high enough
            if (max_class_prob >= conf_threshold_) {
                // Get bounding box coordinates (normalized [0,1])
                float x = output_data[i * dimensions + 0];
                float y = output_data[i * dimensions + 1];
                float w = output_data[i * dimensions + 2];
                float h = output_data[i * dimensions + 3];
                
                // Convert to absolute coordinates (relative to original image)
                float scale_x = static_cast<float>(original_size.width) / static_cast<float>(image_size_);
                float scale_y = static_cast<float>(original_size.height) / static_cast<float>(image_size_);
                
                float xmin = (x - w/2) * scale_x;
                float ymin = (y - h/2) * scale_y;
                float xmax = (x + w/2) * scale_x;
                float ymax = (y + h/2) * scale_y;
                
                // Store detection [xmin, ymin, xmax, ymax, confidence, class_id, class_prob]
                temp_detections.push_back({xmin, ymin, xmax, ymax, confidence, static_cast<float>(class_id), max_class_prob});
            }
        }
    }
    
    return temp_detections;
}

std::vector<std::vector<float>> YoloNode::applyNMS(const std::vector<std::vector<float>>& temp_detections) {
    std::vector<std::vector<float>> detections;
    std::vector<int> indices(temp_detections.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    // Sort indices by confidence (descending)
    std::sort(indices.begin(), indices.end(), [&temp_detections](int i1, int i2) {
        return temp_detections[i1][4] > temp_detections[i2][4];
    });
    
    std::vector<bool> keep(indices.size(), true);
    
    // Apply NMS
    for (size_t i = 0; i < indices.size(); ++i) {
        if (!keep[i]) continue;
        
        const auto& detection_i = temp_detections[indices[i]];
        float area_i = (detection_i[2] - detection_i[0]) * (detection_i[3] - detection_i[1]);
        
        for (size_t j = i + 1; j < indices.size(); ++j) {
            if (!keep[j]) continue;
            
            const auto& detection_j = temp_detections[indices[j]];
            
            // Check if boxes are for the same class
            if (detection_i[5] != detection_j[5]) continue;
            
            // Calculate intersection area
            float xmin = std::max(detection_i[0], detection_j[0]);
            float ymin = std::max(detection_i[1], detection_j[1]);
            float xmax = std::min(detection_i[2], detection_j[2]);
            float ymax = std::min(detection_i[3], detection_j[3]);
            
            float width = std::max(0.0f, xmax - xmin);
            float height = std::max(0.0f, ymax - ymin);
            float intersection_area = width * height;
            
            float area_j = (detection_j[2] - detection_j[0]) * (detection_j[3] - detection_j[1]);
            float union_area = area_i + area_j - intersection_area;
            float iou = intersection_area / union_area;
            
            if (iou > iou_threshold_) {
                keep[j] = false;
            }
        }
    }
    
    // Add kept detections to final result
    for (size_t i = 0; i < indices.size() && detections.size() < max_detections_; ++i) {
        if (keep[i]) {
            detections.push_back(temp_detections[indices[i]]);
        }
    }
    
    return detections;
}

void YoloNode::loadClassNames(const std::string& names_file) {
    std::ifstream file(names_file);
    if (!file.is_open()) {
        ROS_WARN("Could not open class names file: %s", names_file.c_str());
        return;
    }
    cv::Mat& visualization = image;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            class_names_.push_back(line);
        }
    }
    
    ROS_INFO("Loaded %zu class names from %s", class_names_.size(), names_file.c_str());
}

cv::Mat YoloNode::visualizeDetections(cv::Mat& image, const std::vector<std::vector<float>>& detections) {
    cv::Mat visualization = image.clone();
    
    for (const auto& detection : detections) {
        // Extract detection info
        int x1 = static_cast<int>(detection[0]);
        int y1 = static_cast<int>(detection[1]);
        int x2 = static_cast<int>(detection[2]);
        int y2 = static_cast<int>(detection[3]);
        float conf = detection[4];
        int class_id = static_cast<int>(detection[5]);
        
        // Ensure coordinates are within image bounds
        x1 = std::max(0, std::min(x1, visualization.cols - 1));
        y1 = std::max(0, std::min(y1, visualization.rows - 1));
        x2 = std::max(0, std::min(x2, visualization.cols - 1));
        y2 = std::max(0, std::min(y2, visualization.rows - 1));
        
        // Draw rectangle around detection
        cv::rectangle(visualization, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        
        // Create label with class name and confidence
        std::string class_name = (class_id < class_names_.size()) ? class_names_[class_id] : std::to_string(class_id);
        std::stringstream ss;
        ss << class_name << " " << std::fixed << std::setprecision(2) << conf;
        std::string label = ss.str();
        
        // Get size of text for background rectangle
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
        
        // Ensure label is within image bounds
        int text_y = std::max(y1, text_size.height + 5);
        
        // Draw label
        cv::putText(visualization, label, cv::Point(x1, text_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
    
    return visualization;
}


} // namespace object_scout

