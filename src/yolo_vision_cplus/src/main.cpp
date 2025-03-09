#include "YoloNode.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "yolo_node");
    
    // Create node handle with private namespace for parameters
    ros::NodeHandle nh("~");
    
    try {
        // Create and run the YOLO node
        YoloNode node(nh);
        node.spin();
    } 
    catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}