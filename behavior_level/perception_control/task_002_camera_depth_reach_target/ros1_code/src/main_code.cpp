#include "arm_reacher.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_reach_node");
    ros::NodeHandle nh;

    ArmReacher reacher(nh);

    ros::Rate rate(10); // 10 Hz loop
    while (ros::ok()) {
        ros::spinOnce();
        reacher.planAndMove();
        rate.sleep();
    }

    return 0;
}
