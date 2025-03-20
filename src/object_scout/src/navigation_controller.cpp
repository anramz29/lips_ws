#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalID.h>
#include <cmath>
#include <memory>
#include <string>

// Forward declaration
class PoseManager;

/**
 * Utility function to check if a position is safe based on costmap
 */
bool isPositionSafe(const nav_msgs::OccupancyGrid::ConstPtr& costmap, double x, double y) {
    if (!costmap) {
        return false;
    }

    // Convert world coordinates to costmap cell coordinates
    int cell_x = static_cast<int>((x - costmap->info.origin.position.x) / costmap->info.resolution);
    int cell_y = static_cast<int>((y - costmap->info.origin.position.y) / costmap->info.resolution);

    // Check if the coordinates are within the costmap bounds
    if (cell_x < 0 || cell_x >= static_cast<int>(costmap->info.width) ||
        cell_y < 0 || cell_y >= static_cast<int>(costmap->info.height)) {
        return false;
    }

    // Calculate the index in the data array
    int index = cell_y * costmap->info.width + cell_x;

    // Check the cost value
    // 0 = free space, 100 = occupied, -1 = unknown
    return costmap->data[index] < 50; // Allow free and some partially occupied cells
}

/**
 * Handles robot navigation operations and interfaces with move_base
 *
 * This class provides functionality to:
 * 1. Create and send navigation goals
 * 2. Monitor navigation progress
 * 3. Ensure safe navigation using costmaps
 * 4. Navigate to named poses and specific coordinates
 * 5. Handle orientation calculations and normalization
 */
class NavigationController {
public:
    /**
     * Initialize the navigation controller
     *
     * @param robot_name Name of the robot for topic namespacing
     * @param pose_manager Optional PoseManager instance for named poses
     * @param init_node Whether to initialize a ROS node (standalone mode)
     */
    NavigationController(const std::string& robot_name, PoseManager* pose_manager = nullptr, bool init_node = false) 
        : robot_name_(robot_name), pose_manager_(pose_manager), default_navigation_timeout_(90.0) {
        
        if (init_node) {
            ros::init(ros::M_string(), "navigation_controller");
        }

        // Get topic parameters with default values that use robot_name
        std::string move_base_topic;
        std::string costmap_topic;

        ros::NodeHandle nh("~");
        nh.param<std::string>("move_base_topic", move_base_topic, "/" + robot_name_ + "/move_base");
        nh.param<std::string>("costmap_topic", costmap_topic, "/" + robot_name_ + "/move_base/global_costmap/costmap");

        // Create action client
        move_base_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
            move_base_topic, true);
        
        ROS_INFO_STREAM("Waiting for " << robot_name_ << " move_base action server...");
        move_base_client_->waitForServer();
        ROS_INFO("Connected to move_base action server");

        // Setup cancel publisher
        cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>(move_base_topic + "/cancel", 1);

        // Subscribe to costmap
        costmap_sub_ = nh.subscribe(costmap_topic, 1, &NavigationController::costmapCallback, this);

        // Wait for first costmap
        ROS_INFO("Waiting for costmap...");
        ros::Rate rate(10); // 10 Hz
        while (!costmap_ && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Received costmap");

        // Setup TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /**
     * Move to a specific position with optional orientation
     *
     * @param x X coordinate in the map frame
     * @param y Y coordinate in the map frame
     * @param orientation Optional Quaternion orientation
     * @param timeout Navigation timeout in seconds (default: default_navigation_timeout_)
     * @return Success flag
     */
    bool moveToPosition(double x, double y, 
                        const geometry_msgs::Quaternion* orientation = nullptr, 
                        double timeout = 0.0) {
        // Use default timeout if not specified
        if (timeout <= 0.0) {
            timeout = default_navigation_timeout_;
        }

        // Safety check
        auto current_pose = getRobotPose();
        if (!current_pose) {
            ROS_ERROR("Failed to get current robot pose");
            return false;
        }

        if (!isPositionSafe(costmap_, x, y)) {
            ROS_ERROR_STREAM("Position (" << x << ", " << y << ") is in unsafe area!");
            return false;
        }

        // Create navigation goal
        auto goal = createNavigationGoal(x, y, orientation, current_pose->pose);

        // Send the goal
        ROS_INFO_STREAM("Moving to position: x=" << x << ", y=" << y);
        move_base_client_->sendGoal(goal);

        // Wait for result with timeout
        return waitForNavigation(timeout);
    }

    /**
     * Return the robot to a specified position and orientation
     *
     * This method is semantically different from moveToPosition as it indicates
     * an intent to return to a previously visited position.
     *
     * @param x X coordinate to return to
     * @param y Y coordinate to return to
     * @param orientation Orientation to set at the position
     * @param timeout Navigation timeout in seconds (default: default_navigation_timeout_)
     * @return True if navigation succeeded, False otherwise
     */
    bool returnToPosition(double x, double y, 
                         const geometry_msgs::Quaternion* orientation = nullptr, 
                         double timeout = 0.0) {
        ROS_INFO_STREAM("Returning to position: (" << x << ", " << y << ")");
        return moveToPosition(x, y, orientation, timeout);
    }

    /**
     * Rotate the robot in place by the specified angle
     *
     * @param angle_degrees Angle to rotate in degrees
     * @param timeout Rotation timeout in seconds (default: 10.0)
     * @return True if rotation succeeded, False otherwise
     */
    bool rotateInPlace(double angle_degrees, double timeout = 10.0) {
        // Get current pose
        auto current_pose = getRobotPose();
        if (!current_pose) {
            ROS_ERROR("Failed to get current robot pose");
            return false;
        }

        // Convert angle to radians
        double angle_radians = angle_degrees * M_PI / 180.0;

        // Create quaternion for the target orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, angle_radians);
        geometry_msgs::Quaternion orientation = tf2::toMsg(q);

        // Create goal at current position with new orientation
        ROS_INFO_STREAM("Rotating " << angle_degrees << " degrees in place");
        auto goal = createNavigationGoal(
            current_pose->pose.position.x,
            current_pose->pose.position.y,
            &orientation
        );

        // Send the goal
        move_base_client_->sendGoal(goal);

        // Wait for result with timeout
        return waitForNavigation(timeout);
    }

    /**
     * Navigate the robot to a named pose from the pose manager
     *
     * @param pose_name Name of the pose to navigate to
     * @param timeout Navigation timeout in seconds (default: default_navigation_timeout_)
     * @return True if navigation succeeded, False otherwise
     * @throws std::runtime_error If pose_manager is not set or pose_name not found
     */
    bool navigateToNamedPose(const std::string& pose_name, double timeout = 0.0) {
        if (pose_manager_ == nullptr) {
            throw std::runtime_error("PoseManager not set in NavigationController");
        }

        ROS_INFO_STREAM("Navigating to named pose: " << pose_name);

        try {
            // Get pose from the pose manager
            // Note: This would need to be implemented based on your PoseManager class
            geometry_msgs::Pose pose_position;
            // pose_position = pose_manager_->getPose(pose_name);

            // For now, just returning false since we don't have the implementation
            ROS_WARN_STREAM("PoseManager implementation needed for pose '" << pose_name << "'");
            return false;

            // The following code would be used once we have the pose
            /*
            // Extract position
            double x = pose_position.position.x;
            double y = pose_position.position.y;

            // Extract orientation if available
            const geometry_msgs::Quaternion* orientation = &pose_position.orientation;

            // Execute navigation
            bool success = moveToPosition(x, y, orientation, timeout);

            if (!success) {
                ROS_WARN_STREAM("Failed to navigate to pose '" << pose_name << "'");
            }

            return success;
            */
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error navigating to pose '" << pose_name << "': " << e.what());
            return false;
        }
    }

    /**
     * Cancel current navigation goal using both action client and topic
     *
     * Uses redundant cancellation methods to ensure the goal is properly canceled.
     */
    void cancelNavigation() {
        // Cancel via action client
        move_base_client_->cancelAllGoals();
        
        // Cancel via topic for redundancy
        actionlib_msgs::GoalID cancel_msg;
        cancel_pub_.publish(cancel_msg);
        
        ROS_INFO("Sent navigation cancellation commands");
    }

    /**
     * Get current robot pose in map frame
     *
     * @return PoseStamped containing the robot pose or empty optional if failed
     */
    std::optional<geometry_msgs::PoseStamped> getRobotPose() {
        try {
            // Give time for the listener to receive transforms
            ros::Duration(0.5).sleep();

            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                "map", robot_name_ + "/base_link", ros::Time(0), ros::Duration(1.0));

            geometry_msgs::PoseStamped current_pose;
            current_pose.header = transform.header;
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.position.z = transform.transform.translation.z;
            current_pose.pose.orientation = transform.transform.rotation;
            
            return current_pose;
        } catch (const tf2::TransformException& e) {
            ROS_ERROR_STREAM("Failed to get robot pose: " << e.what());
            return std::nullopt;
        }
    }

private:
    std::string robot_name_;
    PoseManager* pose_manager_;
    double default_navigation_timeout_;
    
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    ros::Publisher cancel_pub_;
    ros::Subscriber costmap_sub_;
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * Callback for costmap updates
     *
     * @param msg The costmap message
     */
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        costmap_ = msg;
    }

    /**
     * Create a MoveBaseGoal message for the specified position and orientation
     *
     * @param x X coordinate in the map frame
     * @param y Y coordinate in the map frame
     * @param orientation Optional Quaternion orientation
     * @param current_pose Current robot pose for calculating direction
     * @return The configured goal message
     */
    move_base_msgs::MoveBaseGoal createNavigationGoal(
        double x, double y, 
        const geometry_msgs::Quaternion* orientation = nullptr,
        const geometry_msgs::Pose& current_pose = geometry_msgs::Pose()) {
        
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0;

        // Handle orientation
        if (orientation) {
            // Validate the quaternion
            double norm = std::sqrt(
                orientation->x * orientation->x + 
                orientation->y * orientation->y + 
                orientation->z * orientation->z + 
                orientation->w * orientation->w
            );

            // If the quaternion is nearly zero length or very far from unit length
            if (norm < 0.1) {
                ROS_WARN_STREAM("Quaternion has length close to zero: " << norm << ", using default");
                
                // Use default orientation instead
                if (current_pose.position.x != 0 || current_pose.position.y != 0) {
                    // Calculate angle to target
                    double dx = x - current_pose.position.x;
                    double dy = y - current_pose.position.y;
                    double angle = std::atan2(dy, dx);
                    
                    tf2::Quaternion q;
                    q.setRPY(0, 0, angle);
                    goal.target_pose.pose.orientation = tf2::toMsg(q);
                } else {
                    // Default facing forward
                    tf2::Quaternion q;
                    q.setRPY(0, 0, 0);
                    goal.target_pose.pose.orientation = tf2::toMsg(q);
                }
            } else if (std::abs(norm - 1.0) > 0.01) {
                // Normalize the quaternion if it's not unit length
                ROS_WARN_STREAM("Quaternion not normalized (norm=" << norm << "), normalizing");
                double factor = 1.0 / norm;
                geometry_msgs::Quaternion normalized_quat;
                normalized_quat.x = orientation->x * factor;
                normalized_quat.y = orientation->y * factor;
                normalized_quat.z = orientation->z * factor;
                normalized_quat.w = orientation->w * factor;
                goal.target_pose.pose.orientation = normalized_quat;
            } else {
                // Use the provided orientation as is
                goal.target_pose.pose.orientation = *orientation;
            }
        } else {
            // No orientation provided, calculate one
            if (current_pose.position.x != 0 || current_pose.position.y != 0) {
                // Face the direction of movement
                double dx = x - current_pose.position.x;
                double dy = y - current_pose.position.y;
                double angle = std::atan2(dy, dx);
                
                tf2::Quaternion q;
                q.setRPY(0, 0, angle);
                goal.target_pose.pose.orientation = tf2::toMsg(q);
                
                ROS_INFO_STREAM("Setting orientation to face direction of travel (yaw: " 
                    << angle * 180.0 / M_PI << " degrees)");
            } else {
                // Default facing forward (positive x-axis)
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                goal.target_pose.pose.orientation = tf2::toMsg(q);
            }
        }

        return goal;
    }

    /**
     * Wait for navigation to complete with timeout
     *
     * @param timeout Timeout in seconds
     * @return Success flag
     */
    bool waitForNavigation(double timeout) {
        bool finished_before_timeout = move_base_client_->waitForResult(ros::Duration(timeout));

        if (!finished_before_timeout) {
            ROS_ERROR_STREAM("Navigation timed out after " << timeout << " seconds");
            move_base_client_->cancelGoal();
            return false;
        }

        auto state = move_base_client_->getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);

        if (!success) {
            ROS_ERROR_STREAM("Navigation failed with state: " << state.toString());
        }

        return success;
    }
};

// Main function for standalone execution
int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "navigation_controller");
        ros::NodeHandle nh("~");
        
        std::string robot_name;
        nh.param<std::string>("robot_name", robot_name, "locobot");
        
        // Run as standalone node
        NavigationController controller(robot_name);
        ros::spin();
        
    } catch (ros::Exception& e) {
        ROS_ERROR_STREAM("ROS exception: " << e.what());
        return 1;
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return 1;
    }
    
    return 0;
}