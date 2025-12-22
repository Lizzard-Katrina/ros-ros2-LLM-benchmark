#include "sensor_fusion_visualizer.h"

SensorFusionVisualizer::SensorFusionVisualizer(ros::NodeHandle& nh) {
    camera_sub_ = nh.subscribe("/camera/image_raw", 10, &SensorFusionVisualizer::cameraCallback, this);
    lidar_sub_ = nh.subscribe("/lidar/points", 10, &SensorFusionVisualizer::lidarCallback, this);
    fused_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/fused_markers", 10);

    // TODO: Initialize sensor fusion visualization algorithm (Autoware skeleton)
    // --------------------------
    // 1. Initialize sensor buffers
    // 2. Load fusion parameters
    // 3. Initialize any mapping / visualization modules
    // END
}
