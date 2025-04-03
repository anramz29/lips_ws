#ifndef POSE_MANAGER_HPP
#define POSE_MANAGER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>

class PoseManager
{
public:
    /**
     * Initialize the pose manager
     * 
     * @param poses_config Path to poses YAML configuration file
     */
    PoseManager(const std::string& poses_config);
    
    /**
     * Get a pose by name
     * 
     * @param pose_name Name of the pose in configuration
     * @return The requested pose or an empty pose if not found
     */
    geometry_msgs::Pose getPose(const std::string& pose_name);
    
    /**
     * Get all available pose names
     * 
     * @return List of all pose names
     */
    std::vector<std::string> getAllPoseNames();

private:
    std::string poses_config_;
    std::map<std::string, YAML::Node> poses_;
    
    /**
     * Load poses from YAML config file
     * 
     * @param poses_config Path to poses YAML configuration file
     * @return True if loading was successful, false otherwise
     */
    bool loadPoses(const std::string& poses_config);
};

#endif // POSE_MANAGER_HPP