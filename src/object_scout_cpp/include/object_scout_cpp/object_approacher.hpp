#ifndef OBJECT_APPROACHER_HPP
#define OBJECT_APPROACHER_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "object_scout_cpp/navigation_controller.hpp"

class ObjectApproacher
{
public:
    ObjectApproacher(
        const std::string& robot_name,
        NavigationController* nav_controller,
        const std::string& object_marker_topic,
        const std::string& bbox_depth_topic
    );

    ~ObjectApproacher();

    /**
     * Approach the Object
     */
    bool approachObject(
        double approach_max_depth=1.0,
        double approach_min_depth=0.8
    );

    /**
     * save the pose of the object
     */
    const geometry_msgs::Pose& getObjectPose() const;

private:
    // Core components
    std::string robot_name_;
    NavigationController* nav_controller_;
    std::string object_marker_topic_;
    std::string bbox_depth_topic_;
    ros::Duration navigation_timeout_;

    double max_history_length;
    double min_positions_for_average;
    visualization_msgs::Marker::ConstPtr object_marker_;
    double current_depth_;
    std::vector<double> marker_positions_;

    // ROS subscribers
    ros::Subscriber depth_sub_;
    ros::Subscriber object_marker_sub_;

    // Object pose for storage
    geometry_msgs::Pose object_pose_;

    /**
     * Process object marker messages
     */
    void objectMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    
    /**
     * Process depth information of detected objects
     */
    void depthCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Additional helper methods
    /**
     * Calculate the average position from recent marker history
     * 
     * @return Pair of (avg_x, avg_y) coordinates, or (0,0) if insufficient data
     */
    std::pair<double, double> getAverageMarkerPosition() const;

    /**
     * Calculate the next waypoint for approaching the target
     * 
     * @param target_x Target X coordinate
     * @param target_y Target Y coordinate
     * @param current_pose Current robot pose
     * @param approach_max_depth Maximum approach depth
     * @param approach_min_depth Minimum approach depth
     * @param retry Whether to retry with smaller step if first attempt fails
     * @return Pair of (next_x, next_y) or (0,0) if no safe path found
     */
    std::pair<double, double> calculateApproachWaypoint(
        double target_x, double target_y, 
        const geometry_msgs::Pose& current_pose,
        double approach_max_depth, double approach_min_depth,
        bool retry = true);

    /**
     * Validate depth and marker with sustained detection
     * 
     * @param approach_max_depth Maximum approach depth
     * @param approach_min_depth Minimum approach depth
     * @return True if target depth reached, False if detection failed
     */
    bool checkDepthAndMarker(double approach_max_depth, double approach_min_depth);

    /**
     * Execute a single step in the approach sequence
     * 
     * @param next_x Next waypoint X coordinate
     * @param next_y Next waypoint Y coordinate
     * @param target_x Target X coordinate
     * @param target_y Target Y coordinate
     * @param current_pose Current robot pose
     * @param approach_max_depth Maximum approach depth
     * @param approach_min_depth Minimum approach depth
     * @return True if target reached, False if error occurred
     */
    bool executeApproachStep(
        double next_x, double next_y,
        double target_x, double target_y,
        const geometry_msgs::Pose& current_pose,
        double approach_max_depth, double approach_min_depth);

    /**
     * Check if we've reached target depth during movement
     * 
     * @param approach_max_depth Maximum approach depth
     * @param approach_min_depth Minimum approach depth
     * @return True if target depth reached, False otherwise
     */
    bool checkDepthDuringMovement(double approach_max_depth, double approach_min_depth);

    /**
     * Save the detected object for later reference
     * 
     * @return True if object saved successfully, False otherwise
     */
    bool saveObject();
};

#endif // OBJECT_APPROACHER_HPP