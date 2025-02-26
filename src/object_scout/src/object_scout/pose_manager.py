#!/usr/bin/env python3
import rospy
import yaml
import rospkg
from geometry_msgs.msg import Pose

# Import utilities
from object_scout.utils import load_poses


class PoseManager:
    """
    Manages pose configurations and sequences
    """
    def __init__(self, poses_config, init_node=False):
        """
        Initialize the pose manager
        
        Args:
            poses_config: Path to poses YAML configuration file
            init_node: Whether to initialize a ROS node (standalone mode)
        """
        if init_node:
            rospy.init_node('pose_manager', anonymous=False)
            
        self.poses_config = poses_config
        
        # Load poses from config file
        self.poses = load_poses(poses_config)
        
        if not self.poses:
            rospy.logerr(f"Failed to load poses from config file: {poses_config}")
            raise ValueError(f"No poses found in configuration: {poses_config}")
            
        rospy.loginfo(f"Loaded {len(self.poses)} poses from {poses_config}")
        
    def get_pose(self, pose_name):
        """
        Get a pose by name
        
        Args:
            pose_name: Name of the pose in configuration
            
        Returns:
            Pose: The requested pose or None if not found
        """
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose name: {pose_name}")
            return None
            
        pose = Pose()
        pose.position.x = self.poses[pose_name]['position']['x']
        pose.position.y = self.poses[pose_name]['position']['y']
        return pose
        
    def get_all_pose_names(self):
        """
        Get all available pose names
        
        Returns:
            list: List of all pose names
        """
        return list(self.poses.keys())


if __name__ == "__main__":
    try:
        # Run as standalone node to validate poses
        poses_config = rospy.get_param('~poses_config', '')
        if not poses_config:
            rospack = rospkg.RosPack()
            try:
                pkg_path = rospack.get_path('object_scout')
                poses_config = f"{pkg_path}/config/poses.yaml"
            except rospkg.ResourceNotFound:
                rospy.logerr("Package 'object_scout' not found")
                poses_config = None
        
        if poses_config:
            manager = PoseManager(poses_config, init_node=True)
            rospy.loginfo("Poses loaded successfully. Available poses:")
            for pose_name in manager.get_all_pose_names():
                pose = manager.get_pose(pose_name)
                rospy.loginfo(f"  - {pose_name}: x={pose.position.x:.2f}, y={pose.position.y:.2f}")
            rospy.spin()
        else:
            rospy.logerr("No poses_config parameter provided")
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose manager interrupted")