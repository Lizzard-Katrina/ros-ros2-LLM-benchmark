#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
    ArmController(ros::NodeHandle& nh);

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void planMotion();
    void executeMotion();

private:
    ros::Subscriber cloud_sub_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    geometry_msgs::Pose target_pose_;
    bool new_target_available_;
};

#endif
