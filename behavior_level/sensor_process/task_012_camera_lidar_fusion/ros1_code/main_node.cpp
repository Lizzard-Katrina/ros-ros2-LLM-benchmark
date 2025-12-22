#include "sensor_fusion_visualizer.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion_visualization_node");
    ros::NodeHandle nh;

    SensorFusionVisualizer visualizer(nh);

    ros::spin();
    return 0;
}
