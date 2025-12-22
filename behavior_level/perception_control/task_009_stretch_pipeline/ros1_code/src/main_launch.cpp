#include <ros/ros.h>
#include "stretch_pipeline/perception_node.h"
#include "stretch_pipeline/mapping_node.h"
#include "stretch_pipeline/planner_node.h"
#include "stretch_pipeline/controller_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stretch_pipeline");
    ros::NodeHandle nh;

    stretch_pipeline::PerceptionNode perception(nh);
    stretch_pipeline::MappingNode mapping(nh);
    stretch_pipeline::PlannerNode planner(nh);
    stretch_pipeline::ControllerNode controller(nh);

    ros::spin();
    return 0;
}
