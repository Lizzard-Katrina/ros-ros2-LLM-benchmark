#include "pose_estimator.h"

PoseEstimator::PoseEstimator(ros::NodeHandle& nh) {
    lidar_sub_ = nh.subscribe("/velodyne_points", 100, &PoseEstimator::lidarCallback, this);
    imu_sub_   = nh.subscribe("/imu/data", 100, &PoseEstimator::imuCallback, this);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    map_pub_  = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);

    // TODO: Initialize pose estimation algorithm (LIO-SAM skeleton)
    // --------------------------
    // 1. Initialize point cloud buffers

    // 2. Initialize feature buffers

    // 3. Initialize odometry / IMU states

    // 4. Load algorithm parameters (from ROS param server or default)

    // 5. Initialize optimizer, sliding window, covariance matrices
    // END
}
