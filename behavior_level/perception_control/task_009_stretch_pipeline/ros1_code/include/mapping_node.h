#ifndef STRETCH_PIPELINE_MAPPING_NODE_H
#define STRETCH_PIPELINE_MAPPING_NODE_H

#include <ros/ros.h>
#include <stretch_pipeline/PerceptionMsgs.h> // placeholder
#include <nav_msgs/OccupancyGrid.h>

namespace stretch_pipeline {

class MappingNode {
public:
    MappingNode(ros::NodeHandle& nh);

    void detectionsCallback(const stretch_msgs::ObjectDetections::ConstPtr& msg);

private:
    ros::Subscriber det_sub_;
    ros::Publisher  map_pub_;

    // Internal map structures (e.g., occupancy grid, object table)
};

} // namespace
#endif
