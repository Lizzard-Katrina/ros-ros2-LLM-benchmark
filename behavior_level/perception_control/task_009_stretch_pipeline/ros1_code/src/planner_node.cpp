#include "stretch_pipeline/planner_node.h"
#include <stretch_msgs/PlannedPath.h> // placeholder

using namespace stretch_pipeline;

PlannerNode::PlannerNode(ros::NodeHandle& nh) {
    map_sub_ = nh.subscribe("/local_map", 5, &PlannerNode::mapCallback, this);
    plan_pub_ = nh.advertise<stretch_msgs::PlannedPath>("/planned_path", 5);

    // TODO: Load planner parameters (arm limits, base constraints)
    // END
}

void PlannerNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    // TODO: Compute arm + base plan:
    // - Decide on target pose(s)
    // - Compute arm IK plan (use MoveIt in real implementation)
    // - Compute base path to approach pose
    // - Fill stretch_msgs::PlannedPath with arm+base trajectory and publish
    // END

    // placeholder empty publish
    stretch_msgs::PlannedPath plan;
    plan_pub_.publish(plan);
}
