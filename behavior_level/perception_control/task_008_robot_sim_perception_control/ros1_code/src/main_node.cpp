#include "sim_nav.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, "sim_nav_node");
    ros::NodeHandle nh;

    SimNav node(nh);
    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        node.processPerception();
        node.computeControl();

        rate.sleep();
    }
    return 0;
}
