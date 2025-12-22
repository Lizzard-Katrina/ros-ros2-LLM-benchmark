#include "obstacle_avoider.h"

ObstacleAvoider::ObstacleAvoider(ros::NodeHandle& nh) {
    lidar_sub_ = nh.subscribe("/scan", 10, &ObstacleAvoider::lidarCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Initialize obstacle avoidance parameters
    // --------------------------
    // END
}
