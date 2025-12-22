#include "pose_estimator.h"
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

void PoseEstimator::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
    // Convert ROS msg to PCL
    pcl::fromROSMsg(*lidar_msg, *laserCloud_);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloud_, *laserCloud_, indices);

    // TODO: Feature extraction & local map update
    // - Extract edge/plane features
    // - Update local map with sliding window
    // - Estimate pose increment via LiDAR only
    // END

    // Publish processed point cloud for visualization
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*laserCloud_, cloudMsg);
    cloudMsg.header = lidar_msg->header;
    map_pub_.publish(cloudMsg);
}
