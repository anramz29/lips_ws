#include "object_scout_cpp/navigation_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>

NavigationController::NavigationController(
    const std::string& robot_name,
    const std::string& move_base_topic,
    const std::string& costmap_topic,
    const std::string& move_base_cancel_topic,
    PoseManager* pose_manager
) : 
    robot_name_(robot_name),
    move_base_topic_(move_base_topic),
    costmap_topic_(costmap_topic),
    move_base_cancel_topic_(move_base_cancel_topic),
    pose_manager_(pose_manager),
    default_navigation_timeout_(90.0),
    tf_buffer_(), // Initialize directly here
    costmap_(nullptr)
{
    // Create transform listener with the already initialized buffer
    tf_listener_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));
    
    // Create action client
    client_ = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(move_base_topic_, true));
    
    ROS_INFO_STREAM("Waiting for " << robot_name_ << " move_base action server...");
    client_->waitForServer();
    ROS_INFO("Connected to move_base action server");
    
    // Setup cancel publisher
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(
        move_base_cancel_topic_, 
        1
    );
    
    // Subscribe to costmap
    costmap_sub_ = nh_.subscribe(
        costmap_topic_,
        1,
        &NavigationController::costmapCallback,
        this
    );
    
    // Wait for first costmap
    ROS_INFO("Waiting for costmap...");
    ros::Rate rate(10);
    while (costmap_ == nullptr && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Received costmap");
}

NavigationController::~NavigationController() {
    // Clean up resources if needed
}

void NavigationController::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_ = msg;
}

move_base_msgs::MoveBaseGoal NavigationController::createNavigationGoal(
    double x, double y, 
    const geometry_msgs::Quaternion* orientation,
    const geometry_msgs::Pose* current_pose
) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    
    // Handle orientation
    if (orientation) {
        // Validate quaternion
        double norm = std::sqrt(
            std::pow(orientation->x, 2) + 
            std::pow(orientation->y, 2) + 
            std::pow(orientation->z, 2) + 
            std::pow(orientation->w, 2)
        );
        
        if (norm < 0.1) {
            ROS_WARN_STREAM("Quaternion has length close to zero: " << norm << ", using default");
            // Calculate angle to target if we have current pose
            if (current_pose) {
                double dx = x - current_pose->position.x;
                double dy = y - current_pose->position.y;
                double angle = std::atan2(dy, dx);
                
                tf2::Quaternion quat;
                quat.setRPY(0, 0, angle);
                goal.target_pose.pose.orientation = tf2::toMsg(quat);
            } else {
                // Default facing forward
                tf2::Quaternion quat;
                quat.setRPY(0, 0, 0);
                goal.target_pose.pose.orientation = tf2::toMsg(quat);
            }
        } else if (std::abs(norm - 1.0) > 0.01) {
            // Normalize quaternion
            ROS_WARN_STREAM("Quaternion not normalized (norm=" << norm << "), normalizing");
            geometry_msgs::Quaternion normalized;
            double factor = 1.0 / norm;
            normalized.x = orientation->x * factor;
            normalized.y = orientation->y * factor;
            normalized.z = orientation->z * factor;
            normalized.w = orientation->w * factor;
            goal.target_pose.pose.orientation = normalized;
        } else {
            // Use provided orientation
            goal.target_pose.pose.orientation = *orientation;
        }
    } else {
        // No orientation provided
        if (current_pose) {
            // Face direction of travel
            double dx = x - current_pose->position.x;
            double dy = y - current_pose->position.y;
            double angle = std::atan2(dy, dx);
            
            tf2::Quaternion quat;
            quat.setRPY(0, 0, angle);
            goal.target_pose.pose.orientation = tf2::toMsg(quat);
            
            ROS_INFO_STREAM("Setting orientation to face direction of travel (yaw: " 
                          << angle * 180.0 / M_PI << " degrees)");
        } else {
            // Default facing forward
            tf2::Quaternion quat;
            quat.setRPY(0, 0, 0);
            goal.target_pose.pose.orientation = tf2::toMsg(quat);
        }
    }
    
    return goal;
}

bool NavigationController::moveToPosition(double x, double y, 
                                         const geometry_msgs::Quaternion* orientation, 
                                         double timeout) {
    // Use default timeout if not specified
    if (timeout <= 0.0) {
        timeout = default_navigation_timeout_;
    }
    
    // Safety check
    geometry_msgs::Pose current_pose = getRobotPose();
    if (costmap_ && !isPositionSafe(*costmap_, x, y)) {
        ROS_ERROR_STREAM("Position (" << x << ", " << y << ") is in unsafe area!");
        return false;
    }
    
    // Create navigation goal
    move_base_msgs::MoveBaseGoal goal = createNavigationGoal(x, y, orientation, &current_pose);
    
    // Send the goal
    ROS_INFO_STREAM("Moving to position: x=" << x << ", y=" << y);
    client_->sendGoal(goal);
    
    // Wait for result with timeout
    return waitForNavigation(timeout);
}

bool NavigationController::waitForNavigation(double timeout) {
    bool finished_in_time = client_->waitForResult(ros::Duration(timeout));
    
    if (!finished_in_time) {
        ROS_ERROR_STREAM("Navigation timed out after " << timeout << " seconds");
        client_->cancelGoal();
        return false;
    }
    
    actionlib::SimpleClientGoalState state = client_->getState();
    bool success = state == actionlib::SimpleClientGoalState::SUCCEEDED;
    
    if (!success) {
        ROS_ERROR_STREAM("Navigation failed with state: " << state.toString());
    }
    
    return success;
}

void NavigationController::cancelNavigation() {
    // Cancel via action client
    client_->cancelAllGoals();
    
    // Cancel via topic for redundancy
    actionlib_msgs::GoalID cancel_msg;
    cancel_pub_.publish(cancel_msg);
    ROS_INFO("Sent navigation cancellation commands");
}

geometry_msgs::Pose NavigationController::getRobotPose() {
    geometry_msgs::Pose pose;
    
    try {
        // Allow time for tf to become available
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "map", 
            robot_name_ + "/base_link", 
            ros::Time(0),
            ros::Duration(0.5)
        );
        
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;
        
    } catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Failed to get robot pose: " << ex.what());
    }
    
    return pose;
}

bool NavigationController::isPositionSafe(const nav_msgs::OccupancyGrid& costmap, double x, double y) {
    // Convert world coordinates to costmap cell coordinates
    int cell_x = static_cast<int>((x - costmap.info.origin.position.x) / costmap.info.resolution);
    int cell_y = static_cast<int>((y - costmap.info.origin.position.y) / costmap.info.resolution);
    
    // Check if coordinates are within costmap bounds
    if (cell_x < 0 || cell_x >= costmap.info.width ||
        cell_y < 0 || cell_y >= costmap.info.height) {
        ROS_WARN_STREAM("Position (" << x << ", " << y << ") is outside costmap bounds");
        return false;
    }
    
    // Get cost value at position
    int index = cell_y * costmap.info.width + cell_x;
    int8_t cost = costmap.data[index];
    
    // Cost values: -1 = unknown, 0 = free, 1-99 = cost, 100 = occupied
    if (cost == -1) {
        ROS_WARN_STREAM("Position (" << x << ", " << y << ") is in unknown space");
        return false;
    } else if (cost >= 90) { // Threshold can be adjusted
        ROS_WARN_STREAM("Position (" << x << ", " << y << ") is too close to obstacles (cost: " << static_cast<int>(cost) << ")");
        return false;
    }
    
    return true;
}

bool NavigationController::clearCostmaps() {
    try {
        // Service name for clearing costmaps
        std::string clear_costmaps_service = move_base_topic_ + "/clear_costmaps";
        
        // Wait for service to be available
        ROS_INFO_STREAM("Waiting for " << clear_costmaps_service << " service...");
        if (!ros::service::waitForService(clear_costmaps_service, ros::Duration(5.0))) {
            ROS_ERROR("Service not available within timeout");
            return false;
        }
        
        // Call the service
        std_srvs::Empty srv;
        if (!ros::service::call(clear_costmaps_service, srv)) {
            ROS_ERROR("Failed to call clear_costmaps service");
            return false;
        }
        
        ROS_INFO("Successfully cleared costmaps");
        return true;
        
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("ROS exception: " << e.what());
        return false;
    }
}

bool NavigationController::navigateToNamedPose(const std::string& pose_name, double timeout) {
    // Check if pose manager is available
    if (!pose_manager_) {
        ROS_ERROR("Cannot navigate to named pose: PoseManager not set in NavigationController");
        return false;
    }
    
    // Use default timeout if not specified
    if (timeout <= 0.0) {
        timeout = default_navigation_timeout_;
    }
    
    ROS_INFO_STREAM("Navigating to named pose: " << pose_name);
    
    try {
        // Get pose from the pose manager
        geometry_msgs::Pose pose = pose_manager_->getPose(pose_name);
        
        // Check if pose is valid (simple check - default pose would have all zeros)
        bool is_default_pose = 
            pose.position.x == 0.0 && 
            pose.position.y == 0.0 && 
            pose.position.z == 0.0 &&
            pose.orientation.x == 0.0 &&
            pose.orientation.y == 0.0 &&
            pose.orientation.z == 0.0 &&
            pose.orientation.w == 0.0;
            
        if (is_default_pose) {
            ROS_WARN_STREAM("Pose '" << pose_name << "' not found in pose manager");
            return false;
        }
        
        // Extract position
        double x = pose.position.x;
        double y = pose.position.y;
        
        // Execute navigation with orientation
        const geometry_msgs::Quaternion* orientation_ptr = nullptr;
        
        // Check if orientation is valid (non-zero norm)
        double orientation_norm = std::sqrt(
            std::pow(pose.orientation.x, 2) +
            std::pow(pose.orientation.y, 2) +
            std::pow(pose.orientation.z, 2) +
            std::pow(pose.orientation.w, 2)
        );
        
        if (orientation_norm > 0.1) {
            orientation_ptr = &pose.orientation;
        }
        
        bool success = moveToPosition(x, y, orientation_ptr, timeout);
        
        if (!success) {
            ROS_WARN_STREAM("Failed to navigate to pose '" << pose_name << "'");
        }
        
        return success;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error navigating to pose '" << pose_name << "': " << e.what());
        return false;
    }
}
