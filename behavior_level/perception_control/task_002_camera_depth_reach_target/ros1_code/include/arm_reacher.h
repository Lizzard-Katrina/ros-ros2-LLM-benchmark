#ifndef ARM_REACHER_H
#define ARM_REACHER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ArmReacher {
public:
    ArmReacher(ros::NodeHandle& nh);

    void depthCameraCallback(const sensor_msgs::Image::ConstPtr& img_msg);
    void planAndMove();

private:
    ros::Subscriber depth_camera_sub_;
    geometry_msgs::PoseStamped target_pose_;
    bool target_detected_;

    moveit::planning_interface::MoveGroupInterface* move_group_;
};

#endif
