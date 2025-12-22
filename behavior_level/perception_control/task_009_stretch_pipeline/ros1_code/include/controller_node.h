#ifndef STRETCH_PIPELINE_CONTROLLER_NODE_H
#define STRETCH_PIPELINE_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

namespace stretch_pipeline {

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle& nh);

    void planCallback(const stretch_msgs::PlannedPath::ConstPtr& plan);

private:
    ros::Subscriber plan_sub_;
    ros::Publisher  cmd_pub_;
    ros::Publisher  joint_cmd_pub_;

    // Controller placeholders
};

} // namespace
#endif
