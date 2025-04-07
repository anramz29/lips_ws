#ifndef OBJECT_SCANNER_HPP
#define OBJECT_SCANNER_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <memory>

#include "object_scout_cpp/navigation_controller.hpp"

// Enumeration for scan operation results
enum class ScanResult {
    NO_DETECTION = 0,
    OBJECT_DETECTED = 1,
    ERROR = 2
};

class ObjectScanner {
public:
    /**
     * Initialize the scanner with robot name and navigation controller
     * 
     * @param robot_name Name of the robot for topic namespacing
     * @param nav_controller Instance of NavigationController for rotation commands
     * @param object_mapper_topic Topic for object marker messages
     * @param bbox_depth_topic Topic for bounding box depth information
     */
    ObjectScanner(
        const std::string& robot_name,
        NavigationController* nav_controller,
        const std::string& object_mapper_topic,
        const std::string& bbox_depth_topic
    );
    
    /**
     * Reset object detection state
     */
    void resetDetectionState();
    
    /**
     * Get the list of remaining scan angles
     * 
     * @return List of remaining angles to scan in degrees
     */
    std::vector<double> getRemainingAngles() const;
    
    /**
     * Perform a sequence of rotations to scan for objects
     * 
     * @param rotation_angles List of angles in degrees to scan. If empty, uses default angles.
     * @param desired_class_id Class ID to scan for, or -1 for any class
     * @param complete_scan If true, completes all rotations regardless of detections
     * @return Pair of (ScanResult, remaining_angles) 
     */
    std::pair<ScanResult, std::vector<double>> performScanRotation(
        const std::vector<double>& rotation_angles = std::vector<double>(),
        int desired_class_id = -1,
        bool complete_scan = false
    );
    
private:
    // Core components
    std::string robot_name_;
    NavigationController* nav_controller_;
    std::string object_mapper_topic_;
    std::string bbox_depth_topic_;
    
    // Scanner parameters
    double scan_stabilization_time_;
    ros::Duration required_detection_duration_;
    ros::Duration scan_timeout_;
    ros::Duration rotation_timeout_;
    double max_detection_depth_;
    
    // Default rotation angles in degrees
    std::vector<double> default_rotation_angles_;
    
    // Scanner state
    bool scanning_in_progress_;
    visualization_msgs::Marker::ConstPtr object_marker_;
    bool object_detected_;
    double current_depth_;
    int current_class_id_;
    std::vector<double> remaining_angles_;
    
    // ROS subscribers
    ros::Subscriber object_marker_sub_;
    ros::Subscriber depth_sub_;
    
    /**
     * Process object marker messages
     */
    void objectMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    
    /**
     * Process depth information of detected objects
     */
    void depthCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    /**
     * Scan for objects at current position with sustained detection requirement
     * 
     * @param angle Current rotation angle for logging
     * @param desired_class_id Class ID to scan for, or -1 to accept any class
     * @return True if object detected with sustained requirement, False otherwise
     */
    bool scanWithSustainedDetection(double angle, int desired_class_id = -1);
    
    /**
     * Perform a single rotation and scan for objects
     * 
     * @param current_pose Current robot pose
     * @param angle Rotation angle in degrees
     * @param rotation_index Index of the current rotation
     * @param total_rotations Total number of rotations
     * @param desired_class_id Class ID to scan for, or -1 to accept any class
     * @return Result of the scan operation
     */
    ScanResult performSingleRotationScan(
        const geometry_msgs::Pose& current_pose,
        double angle,
        int rotation_index,
        int total_rotations,
        int desired_class_id
    );
};

#endif // OBJECT_SCANNER_HPP