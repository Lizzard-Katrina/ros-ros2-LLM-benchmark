#include "pose_estimator.h"
#include <Eigen/Dense>

void PoseEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    Eigen::Vector3d angVel(imu_msg->angular_velocity.x,
                           imu_msg->angular_velocity.y,
                           imu_msg->angular_velocity.z);
    Eigen::Vector3d linAcc(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);

    // TODO: Integrate IMU with LiDAR to update pose
    // - Predict pose increment using IMU
    // - Fuse with LiDAR-based odometry
    // - Update position_, velocity_, orientation_
    // END

    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = imu_msg->header.stamp;
    odomMsg.header.frame_id = "odom";
    odom_pub_.publish(odomMsg);
}
