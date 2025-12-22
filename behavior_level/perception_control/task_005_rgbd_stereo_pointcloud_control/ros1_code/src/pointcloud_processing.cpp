#include "pointcloud_controller.h"
#include <pcl/filters/filter.h>

void PointCloudController::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *latest_cloud_);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*latest_cloud_, *latest_cloud_, indices);

    // TODO: Process point cloud for obstacle detection
    // --------------------------
    // - Analyze point distances
    // - Set obstacle_detected_ = true if points are closer than threshold
    // END
}
