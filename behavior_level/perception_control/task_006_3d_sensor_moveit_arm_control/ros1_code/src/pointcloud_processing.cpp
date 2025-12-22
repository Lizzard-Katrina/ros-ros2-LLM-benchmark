#include "arm_controller.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

void ArmController::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // TODO: Extract target position from point cloud
    // --------------------------
    // - Compute centroid or object pose
    // - Update target_pose_
    // - Set new_target_available_ = true
    // END
}
