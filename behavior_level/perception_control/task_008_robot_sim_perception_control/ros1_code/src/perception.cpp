#include "sim_nav.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void SimNav::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
    latest_image_ = *msg;
    image_received_ = true;
}

void SimNav::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_lidar_ = *msg;
    lidar_received_ = true;
}

void SimNav::processPerception() {
    if (!image_received_ || !lidar_received_) return;

    // TODO: Process simulated camera & lidar data to detect obstacles
    // ---------------------------------------------------------------
    // - Convert image to CV format
    // - Detect colored regions or edges (very simple)
    // - Analyze LiDAR ranges for close obstacles
    // - Set internal state or flags for controller
    // END
}
