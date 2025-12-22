#include "pointcloud_controller.h"

PointCloudController::PointCloudController(ros::NodeHandle& nh) {
    cloud_sub_ = nh.subscribe("/camera/depth/points", 10, &PointCloudController::pointCloudCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Initialize motion control parameters
    // --------------------------
    // END
}
