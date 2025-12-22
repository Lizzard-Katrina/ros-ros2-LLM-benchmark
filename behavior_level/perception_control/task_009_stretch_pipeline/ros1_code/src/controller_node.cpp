#include "stretch_pipeline/controller_node.h"
#include <stretch_msgs/ControllerCmd.h> // placeholder
#include <trajectory_msgs/JointTrajectory.h>

using namespace stretch_pipeline;

ControllerNode::ControllerNode(ros::NodeHandle& nh) {
    plan_sub_ = nh.subscribe("/planned_path", 5, &ControllerNode::planCallback, this);
    cmd_pub_  = nh.advertise<stretch_msgs::ControllerCmd>("/stretch_controller/cmd", 5);
    joint_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 5);

    // TODO: Initialize controller parameters (safety limits etc.)
    // END
}

void ControllerNode::planCallback(const stretch_msgs::PlannedPath::ConstPtr& msg) {
    // TODO: Execute plan:
    // - Convert planned path into joint trajectories and base commands
    // - Send to appropriate topics (/arm_controller/command and base controller)
    // - Perform safety checks, monitor feedback
    // END

    // placeholder: publish an empty ControllerCmd for now
    stretch_msgs::ControllerCmd cmd;
    cmd_pub_.publish(cmd);
}
