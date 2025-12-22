#include "nav_system.h"

void NavSystem::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_lidar_ = *msg;
    lidar_received_ = true;
}

void NavSystem::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
    latest_image_ = *msg;
    camera_received_ = true;
}

void NavSystem::fuseSensors() {
    if (!lidar_received_ || !camera_received_) return;

    // TODO: Fuse LiDAR and camera information
    // --------------------------
    // - Convert camera image to CV format (cv_bridge)
    // - Detect obstacles or texture cues
    // - Combine with LiDAR ranges
    // - Produce fused obstacle map or costmap representation
    // END
}
