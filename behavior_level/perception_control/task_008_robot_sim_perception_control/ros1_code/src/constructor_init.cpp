#include "sim_nav.h"
#include <ros/ros.h>

SimNav::SimNav(ros::NodeHandle& nh)
    : image_received_(false), lidar_received_(false)
{
    camera_sub_ = nh.subscribe("/camera/image_raw", 10,
                               &SimNav::cameraCallback, this);
    lidar_sub_  = nh.subscribe("/scan", 10,
                               &SimNav::lidarCallback, this);

    cmd_pub_    = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Load simulation control parameters
    // ----------------------------------------
    // - Desired speed
    // - Obstacle distance thresholds
    // - PID / P-controller gains
    // END
}
