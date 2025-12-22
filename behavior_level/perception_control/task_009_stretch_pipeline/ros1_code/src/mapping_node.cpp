#include "stretch_pipeline/mapping_node.h"
#include <stretch_msgs/LocalMap.h> // placeholder
#include <nav_msgs/OccupancyGrid.h>

using namespace stretch_pipeline;

MappingNode::MappingNode(ros::NodeHandle& nh) {
    det_sub_ = nh.subscribe("/detected_objects", 10, &MappingNode::detectionsCallback, this);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 5);

    // TODO: Load mapping parameters (map size, resolution)
    // END
}

void MappingNode::detectionsCallback(const stretch_msgs::ObjectDetections::ConstPtr& msg) {
    // TODO: Update local map with incoming detections
    // - Insert or update object poses
    // - Optionally integrate with basic occupancy grid
    // - Publish nav_msgs::OccupancyGrid on /local_map
    // END

    // placeholder publish empty map
    nav_msgs::OccupancyGrid map;
    map_pub_.publish(map);
}
