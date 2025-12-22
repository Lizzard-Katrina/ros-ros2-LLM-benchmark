#ifndef STRETCH_PIPELINE_PLANNER_NODE_H
#define STRETCH_PIPELINE_PLANNER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace stretch_pipeline {

class PlannerNode {
public:
    PlannerNode(ros::NodeHandle& nh);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

private:
    ros::Subscriber map_sub_;
    ros::Publisher  plan_pub_;

    // Planner config & state
};

} // namespace
#endif
