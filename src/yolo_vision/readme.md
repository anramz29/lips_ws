# YOLO Vision Package

This ROS package provides object detection and segmentation capabilities using YOLO (You Only Look Once) algorithms. It enables robots to detect objects, perform segmentation, and extract keypoints from images.

## Overview

The YOLO Vision package includes:
- Object detection nodes for bounding box detection
- Segmentation capabilities for pixel-wise object identification
- Keypoint detection for identifying specific features on objects
- Distance calculation and object mapping functionality

## Structure

- **launch/** - Launch files for object detection and segmentation
- **models/** - Pre-trained YOLO models:
    - `box_and_grasshopper_segmentation.pt` - Segmentation model for boxes and grasshoppers
    - `grasshopper_and_boxes_keypoint.pt` - Keypoint detection model
    - `simple_bbox_detection.pt` - Simple bounding box detection model
    - `yolo11n.pt` - YOLOv11-nano model
- **rviz/** - RViz configuration files for visualization
- **scripts/** - Python implementation of nodes
- **src/** - C++ implementation of nodes

## Nodes

### Object Detection
- `yolo_node` (Python/C++) - Core object detection using YOLO
- `yolo_keypoint_detection_node.py` - Detects keypoints on objects

### Object Mapping
- `object_mapper_node` (Python/C++) - Maps detected objects in the environment

### Distance Calculation
- `distance_node` (Python/C++) - Calculates distances to detected objects

## Usage

```bash
roslaunch yolo_vision [lanuch file] [params]
```

For more information refer to the launch files in the [launch directory](/src/yolo_vision/launch/)



## Dependencies

- ROS (Robot Operating System)
- PyTorch
- OpenCV
- CUDA (recommended for optimal performance)

## License

This repository is open-source and available under the MIT License.

## Maintainer

- [Adrián Noé Ramírez](https://github.com/anramz29)