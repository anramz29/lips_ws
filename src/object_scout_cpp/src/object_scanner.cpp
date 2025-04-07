#include "object_scout_cpp/object_scanner.hpp"
#include <cmath>

ObjectScanner::ObjectScanner(
    const std::string& robot_name,
    NavigationController* nav_controller,
    const std::string& object_mapper_topic,
    const std::string& bbox_depth_topic
) :
    robot_name_(robot_name),
    nav_controller_(nav_controller),
    object_mapper_topic_(object_mapper_topic),
    bbox_depth_topic_(bbox_depth_topic),
    scan_stabilization_time_(1.0),
    required_detection_duration_(2.0),
    scan_timeout_(3.0),
    rotation_timeout_(45.0),
    max_detection_depth_(10.0),
    scanning_in_progress_(false),
    object_detected_(false),
    current_depth_(0.0),
    current_class_id_(-1)
{
    // Initialize default rotation angles
    default_rotation_angles_ = {0, 45, 90, 135, 180, 225, 270, 315};
    
    // Setup ROS subscribers
    ros::NodeHandle nh;
    
    // Object marker subscription
    object_marker_sub_ = nh.subscribe(
        object_mapper_topic_,
        1,
        &ObjectScanner::objectMarkerCallback,
        this
    );
    
    // Depth subscription
    depth_sub_ = nh.subscribe(
        bbox_depth_topic_,
        1,
        &ObjectScanner::depthCallback,
        this
    );
    
    ROS_INFO_STREAM("ObjectScanner initialized for robot: " << robot_name_);
}

void ObjectScanner::resetDetectionState() {
    object_detected_ = false;
    object_marker_.reset();
}

std::vector<double> ObjectScanner::getRemainingAngles() const {
    return remaining_angles_;
}

void ObjectScanner::objectMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg) {
    if (scanning_in_progress_) {
        if (msg->header.frame_id != "map") {
            ROS_WARN_STREAM("Marker not in map frame: " << msg->header.frame_id);
            return;
        }
        
        // Store the validated marker
        object_marker_ = msg;
    }
}

void ObjectScanner::depthCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    size_t data_length = msg->data.size();
    
    // Check for minimum data length required by new format
    if (data_length >= 8) {
        int n_boxes = static_cast<int>(msg->data[0]);
        if (n_boxes < 1) {
            ROS_DEBUG("No boxes detected in depth message");
            return;
        }
        
        // Extract depth from the new format - position 7
        current_depth_ = msg->data[7];
        
        // Extract class ID from the new format - position 1
        current_class_id_ = static_cast<int>(msg->data[1]);
    }
}

bool ObjectScanner::scanWithSustainedDetection(double angle, int desired_class_id) {
    ROS_INFO_STREAM("Starting object detection at " << angle << " degrees...");
    
    // Variables for sustained detection
    ros::Time detection_start;
    bool has_detection_start = false;
    ros::Time scan_start = ros::Time::now();
    
    // Continue scanning until timeout
    while ((ros::Time::now() - scan_start) < scan_timeout_ && ros::ok()) {
        // Check if we have a marker
        if (object_marker_) {
            // Accept any class ID if desired_class_id is -1, otherwise check for match
            bool class_match = (desired_class_id == -1) || (current_class_id_ == desired_class_id);
            
            if (class_match) {
                ROS_INFO_STREAM_THROTTLE(5.0, "Detected object with class ID " << current_class_id_);
                
                // Start or continue timing the sustained detection
                if (!has_detection_start) {
                    detection_start = ros::Time::now();
                    has_detection_start = true;
                    ROS_INFO("Potential object detected, timing sustained detection...");
                } else {
                    // Check if we've maintained detection long enough
                    ros::Duration elapsed = ros::Time::now() - detection_start;
                    if (elapsed >= required_detection_duration_) {
                        // Validate detection with depth check
                        if (current_depth_ > 0 && current_depth_ < max_detection_depth_) {
                            object_detected_ = true;
                            ROS_INFO_STREAM("Object detected at " << angle << " with class id " << current_class_id_);
                            return true;
                        }
                    }
                }
            } else {
                // Detected object does not match desired class, reset timer
                ROS_WARN_STREAM_THROTTLE(2.0, "Detected object with incorrect class ID " << current_class_id_);
                has_detection_start = false;
            }
        } else {
            // Lost detection, reset the timer
            if (has_detection_start) {
                ROS_WARN("Lost detection during sustainment period, resetting timer");
                has_detection_start = false;
            }
        }
        
        ros::Duration(0.1).sleep();  // Check at 10Hz
    }
    
    ROS_INFO_STREAM("No sustained detection at " << angle << " degrees within timeout");
    return false;
}

ScanResult ObjectScanner::performSingleRotationScan(
    const geometry_msgs::Pose& current_pose,
    double angle,
    int rotation_index,
    int total_rotations,
    int desired_class_id
) {
    // Reset object detection status for this rotation
    object_marker_.reset();
    object_detected_ = false;
    
    // Calculate orientation quaternion
    double yaw = angle * M_PI / 180.0;  // Convert to radians
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    geometry_msgs::Quaternion orientation = tf2::toMsg(tf_quat);
    
    // Send rotation goal
    ROS_INFO_STREAM("Executing rotation " << (rotation_index + 1) << "/" << total_rotations << " (" << angle << " degrees)");
    
    // Attempt to rotate using navigation controller
    bool rotation_success = nav_controller_->moveToPosition(
        current_pose.position.x,
        current_pose.position.y,
        &orientation,
        rotation_timeout_.toSec()
    );
    
    if (!rotation_success) {
        ROS_WARN_STREAM("Rotation goal failed for angle " << angle);
        return ScanResult::ERROR;
    }
    
    // Rotation complete, wait for camera to stabilize
    ROS_INFO_STREAM("Rotation to " << angle << " degrees complete, waiting " << scan_stabilization_time_ << "s for stabilization...");
    ros::Duration(scan_stabilization_time_).sleep();
    
    // Clear any previous detection that might be from motion blur
    object_marker_.reset();
    
    // Scan for objects with sustained detection requirement
    bool detected = scanWithSustainedDetection(angle, desired_class_id);
    
    // Convert boolean result to ScanResult enum
    return detected ? ScanResult::OBJECT_DETECTED : ScanResult::NO_DETECTION;
}

std::pair<ScanResult, std::vector<double>> ObjectScanner::performScanRotation(
    const std::vector<double>& rotation_angles,
    int desired_class_id,
    bool complete_scan
) {
    // Use default angles if none provided
    std::vector<double> angles_to_use;
    if (rotation_angles.empty()) {
        angles_to_use = default_rotation_angles_;
    } else {
        angles_to_use = rotation_angles;
    }
    
    ROS_INFO_STREAM("Starting scan rotation sequence with " << angles_to_use.size() << " angles");
    ROS_INFO_STREAM("Scan mode: " << (complete_scan ? "Complete scan" : "Early exit on detection"));
    
    // Set scanning flag to true and initialize state
    scanning_in_progress_ = true;
    object_detected_ = false;
    remaining_angles_ = angles_to_use;
    bool found_object = false;
    
    // Get current position
    geometry_msgs::Pose current_pose = nav_controller_->getRobotPose();
    
    // Perform the rotations one by one
    for (size_t i = 0; i < angles_to_use.size(); ++i) {
        // Get the current angle to process (always the first in remaining list)
        if (remaining_angles_.empty()) {
            ROS_WARN("No remaining angles to scan");
            break;
        }
        
        double current_angle = remaining_angles_[0];
        ROS_INFO_STREAM("Processing angle " << (i + 1) << "/" << angles_to_use.size() << ": " << current_angle << " degrees");
        
        // Perform the scan at this angle
        ScanResult result = performSingleRotationScan(
            current_pose, 
            current_angle, 
            i, 
            angles_to_use.size(),
            desired_class_id
        );
        
        // Remove this angle from remaining angles AFTER processing it
        remaining_angles_.erase(remaining_angles_.begin());
        ROS_INFO_STREAM("Remaining angles after processing " << current_angle << " degrees: " << remaining_angles_.size());
        
        // If object detected, either return early or continue based on complete_scan flag
        if (result == ScanResult::OBJECT_DETECTED) {
            found_object = true;
            ROS_INFO_STREAM("Object detected during scan rotation at angle " << current_angle);
            if (!complete_scan) {
                scanning_in_progress_ = false;
                return std::make_pair(ScanResult::OBJECT_DETECTED, remaining_angles_);
            }
            // If complete_scan is true, continue to next angle
        }
        
        // If error occurred, log and continue to next angle
        if (result == ScanResult::ERROR) {
            ROS_WARN_STREAM("Error during scan at angle " << current_angle << ", continuing to next angle");
        }
        
        // Brief pause between rotations
        if (i < angles_to_use.size() - 1) {  // Don't pause after the last rotation
            ros::Duration(0.5).sleep();
        }
    }
    
    // End of rotation sequence
    scanning_in_progress_ = false;
    
    if (found_object) {
        ROS_INFO("Completed scan rotation sequence, detected objects at one or more angles");
        return std::make_pair(ScanResult::OBJECT_DETECTED, remaining_angles_);
    } else {
        ROS_INFO("Completed scan rotation sequence, no objects detected");
        return std::make_pair(ScanResult::NO_DETECTION, remaining_angles_);
    }
}