#include "object_scout_cpp/pose_manager.hpp"
#include <fstream>

PoseManager::PoseManager(const std::string& poses_config) 
    : poses_config_(poses_config)
{
    if (!loadPoses(poses_config)) {
        ROS_ERROR_STREAM("Failed to load poses from config file: " << poses_config);
        throw std::runtime_error("No poses found in configuration");
    }
    
    ROS_INFO_STREAM("Loaded " << poses_.size() << " poses from " << poses_config);
}

bool PoseManager::loadPoses(const std::string& poses_config)
{
    try {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(poses_config);
        
        // Check for locations node
        if (!config["locations"]) {
            ROS_ERROR("No 'locations' section in YAML file");
            return false;
        }
        
        // Extract locations
        YAML::Node locations = config["locations"];
        if (!locations.IsMap()) {
            ROS_ERROR("'locations' is not a map in YAML file");
            return false;
        }
        
        // Store each location
        for (const auto& location : locations) {
            std::string name = location.first.as<std::string>();
            poses_[name] = location.second;
        }
        
        return !poses_.empty();
        
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Error parsing YAML: " << e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error loading config: " << e.what());
        return false;
    }
}

geometry_msgs::Pose PoseManager::getPose(const std::string& pose_name)
{
    geometry_msgs::Pose pose;
    
    if (poses_.find(pose_name) == poses_.end()) {
        ROS_ERROR_STREAM("Unknown pose name: " << pose_name);
        return pose;
    }
    
    try {
        auto& pose_node = poses_[pose_name];
        
        if (pose_node["position"] && pose_node["position"]["x"] && pose_node["position"]["y"]) {
            pose.position.x = pose_node["position"]["x"].as<double>();
            pose.position.y = pose_node["position"]["y"].as<double>();
            
            // Optional z coordinate
            if (pose_node["position"]["z"]) {
                pose.position.z = pose_node["position"]["z"].as<double>();
            }
            
            // Optional orientation
            if (pose_node["orientation"]) {
                if (pose_node["orientation"]["x"]) pose.orientation.x = pose_node["orientation"]["x"].as<double>();
                if (pose_node["orientation"]["y"]) pose.orientation.y = pose_node["orientation"]["y"].as<double>();
                if (pose_node["orientation"]["z"]) pose.orientation.z = pose_node["orientation"]["z"].as<double>();
                if (pose_node["orientation"]["w"]) pose.orientation.w = pose_node["orientation"]["w"].as<double>();
            } else {
                // Default orientation (identity quaternion)
                pose.orientation.w = 1.0;
            }
        } else {
            ROS_ERROR_STREAM("Pose " << pose_name << " has invalid format");
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error extracting pose data: " << e.what());
    }
    
    return pose;
}

std::vector<std::string> PoseManager::getAllPoseNames()
{
    std::vector<std::string> names;
    names.reserve(poses_.size());
    
    for (const auto& pose_pair : poses_) {
        names.push_back(pose_pair.first);
    }
    
    return names;
}
