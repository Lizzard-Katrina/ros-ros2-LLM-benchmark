#include "nav_system.h"
#include <ros/ros.h>

NavSystem::NavSystem(ros::NodeHandle& nh)
    : lidar_received_(false), camera_received_(false)
{
    lidar_sub_ = nh.subscribe("/scan", 10, &NavSystem::lidarCallback, this);
    camera_sub_ = nh.subscribe("/camera/rgb/image_raw", 10, &NavSystem::cameraCallback, this);

    cmd_pub_  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("/planned_path", 10);

    // TODO: Load any navigation parameters
    // --------------------------
    // - Load robot radius
    // - Load speed limits
    // - Load costmap parameters
    // END
}
