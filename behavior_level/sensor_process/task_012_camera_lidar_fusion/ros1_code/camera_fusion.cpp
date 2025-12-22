#include "sensor_fusion_visualizer.h"
#include <visualization_msgs/MarkerArray.h>

void SensorFusionVisualizer::cameraCallback(const sensor_msgs::Image::ConstPtr& img_msg) {

    // TODO: Fuse camera data with latest LiDAR cloud for visualization
    // --------------------------
    // - Project LiDAR points onto camera image plane if enabled
    // - Create fused markers for detected objects / points
    // - Update visualization messages
    // END
}

// lidarCallback 完整保留
void SensorFusionVisualizer::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
    latest_cloud_ = *lidar_msg;

    // Fuse LiDAR cloud with latest camera image
    // For this file, we assume simple passthrough for nontodos
    fused_marker_pub_.publish(visualization_msgs::MarkerArray());
}
