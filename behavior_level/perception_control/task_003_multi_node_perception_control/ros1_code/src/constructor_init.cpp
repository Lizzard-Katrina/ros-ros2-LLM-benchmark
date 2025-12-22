#include "multi_node_controller.h"

MultiNodeController::MultiNodeController(ros::NodeHandle& nh) {
    sensor_sub_ = nh.subscribe("/scan", 10, &MultiNodeController::sensorCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Initialize multi-node processing system
    // --------------------------
    // END
}
