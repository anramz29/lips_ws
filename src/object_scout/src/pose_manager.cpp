#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <fstream>

/**
 * @brief Header section for PoseManager
 */
#ifndef OBJECT_SCOUT_POSE_MANAGER_H
#define OBJECT_SCOUT_POSE_MANAGER_H

namespace object_scout {

/**
 * @brief Manages pose configurations and sequences
 */
class PoseManager {
public:
    /**
     * @brief Initialize the pose manager
     * @param poses_config Path to poses YAML configuration file
     * @param init_node Whether to initialize a ROS node (standalone mode)
     */
    PoseManager(const std::string& poses_config, bool init_node = false);

    /**
     * @brief Get a pose by name
     * @param pose_name Name of the pose in configuration
     * @return The requested pose or nullopt if not found
     */
    std::optional<geometry_msgs::Pose> getPose(const std::string& pose_name) const;

    /**
     * @brief Get all available pose names
     * @return List of all pose names
     */
    std::vector<std::string> getAllPoseNames() const;

private:
    std::string poses_config_;
    std::unordered_map<std::string, YAML::Node> poses_;

    /**
     * @brief Load poses from a YAML configuration file
     * @param config_file Path to the configuration file
     * @return Success flag
     */
    bool loadPoses(const std::string& config_file);
};

} // namespace object_scout

#endif // OBJECT_SCOUT_POSE_MANAGER_H

/**
 * @brief Implementation section for PoseManager
 */
namespace object_scout {

/**
 * @brief Helper function to load poses from a YAML file
 */
static std::unordered_map<std::string, YAML::Node> loadPosesFromYaml(const std::string& config_file) {
    std::unordered_map<std::string, YAML::Node> poses;
    
    try {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Check if the file contains a 'poses' node
        if (config["poses"]) {
            YAML::Node poses_node = config["poses"];
            
            // Extract each pose
            for (const auto& pose_entry : poses_node) {
                std::string pose_name = pose_entry.first.as<std::string>();
                poses[pose_name] = pose_entry.second;
            }
        } else {
            ROS_ERROR_STREAM("No 'poses' section found in config file: " << config_file);
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error loading poses from file: " << e.what());
    }
    
    return poses;
}

PoseManager::PoseManager(const std::string& poses_config, bool init_node) : poses_config_(poses_config) {
    if (init_node) {
        ros::init(ros::M_string(), "pose_manager", ros::init_options::AnonymousName);
    }
    
    // Load poses from config file
    if (!loadPoses(poses_config_)) {
        ROS_ERROR_STREAM("Failed to load poses from config file: " << poses_config_);
        throw std::runtime_error("No poses found in configuration: " + poses_config_);
    }
    
    ROS_INFO_STREAM("Loaded " << poses_.size() << " poses from " << poses_config_);
}

bool PoseManager::loadPoses(const std::string& config_file) {
    poses_ = loadPosesFromYaml(config_file);
    return !poses_.empty();
}

std::optional<geometry_msgs::Pose> PoseManager::getPose(const std::string& pose_name) const {
    auto it = poses_.find(pose_name);
    if (it == poses_.end()) {
        ROS_ERROR_STREAM("Unknown pose name: " << pose_name);
        return std::nullopt;
    }
    
    geometry_msgs::Pose pose;
    try {
        const YAML::Node& pose_node = it->second;
        pose.position.x = pose_node["position"]["x"].as<double>();
        pose.position.y = pose_node["position"]["y"].as<double>();
        
        // If orientation is specified, set it
        if (pose_node["orientation"]) {
            if (pose_node["orientation"]["x"] && pose_node["orientation"]["y"] && 
                pose_node["orientation"]["z"] && pose_node["orientation"]["w"]) {
                pose.orientation.x = pose_node["orientation"]["x"].as<double>();
                pose.orientation.y = pose_node["orientation"]["y"].as<double>();
                pose.orientation.z = pose_node["orientation"]["z"].as<double>();
                pose.orientation.w = pose_node["orientation"]["w"].as<double>();
            }
        } else {
            // Default to identity quaternion
            pose.orientation.w = 1.0;
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error parsing pose '" << pose_name << "': " << e.what());
        return std::nullopt;
    }
    
    return pose;
}

std::vector<std::string> PoseManager::getAllPoseNames() const {
    std::vector<std::string> names;
    names.reserve(poses_.size());
    
    for (const auto& pair : poses_) {
        names.push_back(pair.first);
    }
    
    return names;
}

} // namespace object_scout

/**
 * @brief Main function for standalone execution
 */
int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "pose_manager");
        ros::NodeHandle nh("~");
        
        // Get poses config parameter
        std::string poses_config;
        nh.param<std::string>("poses_config", poses_config, "");
        
        if (poses_config.empty()) {
            try {
                std::string pkg_path = ros::package::getPath("object_scout");
                poses_config = pkg_path + "/config/poses.yaml";
            } catch (const std::exception& e) {
                ROS_ERROR("Package 'object_scout' not found");
                return 1;
            }
        }
        
        if (!poses_config.empty()) {
            object_scout::PoseManager manager(poses_config);
            ROS_INFO("Poses loaded successfully. Available poses:");
            
            for (const auto& pose_name : manager.getAllPoseNames()) {
                auto pose = manager.getPose(pose_name);
                if (pose) {
                    ROS_INFO_STREAM("  - " << pose_name << ": x=" << pose->position.x 
                                    << ", y=" << pose->position.y);
                }
            }
            
            ros::spin();
        } else {
            ROS_ERROR("No poses_config parameter provided");
            return 1;
        }
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("ROS exception: " << e.what());
        return 1;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return 1;
    }
    
    return 0;
}