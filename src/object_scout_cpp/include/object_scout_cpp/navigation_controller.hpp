#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalID.h>
#include "object_scout_cpp/pose_manager.hpp"

class NavigationController
{
public:
    NavigationController(
        const std::string& robot_name,
        const std::string& move_base_topic,
        const std::string& costmap_topic,
        const std::string& move_base_cancel_topic,
        PoseManager* pose_manager = nullptr
    );
    
    ~NavigationController();

    // Navigation methods
    bool moveToPosition(
        double x,
        double y,
        const geometry_msgs::Quaternion* orientation = nullptr,
        double timeout = 0.0
    );

    bool waitForNavigationResult(double timeout = 0.0);
    
    bool navigateToNamedPose(
        const std::string& pose_name,
        double timeout = 0.0
    );

    geometry_msgs::Pose getRobotPose();
    bool isPositionSafe(const nav_msgs::OccupancyGrid& costmap, double x, double y);
    bool clearCostmaps();
    void cancelNavigation();
    
    // Set pose manager after construction if needed
    void setPoseManager(PoseManager* pose_manager) {
        pose_manager_ = pose_manager;
    }

private:
    // Move base action client
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    std::unique_ptr<MoveBaseClient> client_;
    
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Configuration
    std::string robot_name_;
    std::string move_base_topic_;
    std::string costmap_topic_;
    std::string move_base_cancel_topic_;
    double default_navigation_timeout_;
    
    // Pose manager
    PoseManager* pose_manager_;
    
    // ROS interfaces
    ros::Publisher cancel_pub_;
    ros::Subscriber costmap_sub_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // State
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    
    // Callbacks
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    // Helper methods
    move_base_msgs::MoveBaseGoal createNavigationGoal(
        double x, double y, 
        const geometry_msgs::Quaternion* orientation = nullptr,
        const geometry_msgs::Pose* current_pose = nullptr
    );
    
    bool waitForNavigation(double timeout);
};

#endif // NAVIGATION_CONTROLLER_HPP