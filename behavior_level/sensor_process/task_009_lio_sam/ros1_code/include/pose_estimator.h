#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class PoseEstimator {
public:
    PoseEstimator(ros::NodeHandle& nh);

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

private:
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher map_pub_;

    // TODO: Add internal data structures for pose estimation
    // END
};

#endif
