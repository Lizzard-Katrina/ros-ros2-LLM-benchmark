#include "sensor_fusion_visualizer.h"
#include <visualization_msgs/MarkerArray.h>

void SensorFusionVisualizer::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {

    // TODO: Fuse LiDAR cloud with latest camera image
    // --------------------------
    // - Optionally filter points
    // - Prepare for projection / marker creation
    // - Update visualization messages
    // END
}

// cameraCallback 完整保留
void SensorFusionVisualizer::cameraCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
    latest_image_ = *img_msg;

    // Fuse camera data with latest LiDAR cloud
    fused_marker_pub_.publish(visualization_msgs::MarkerArray());
}
