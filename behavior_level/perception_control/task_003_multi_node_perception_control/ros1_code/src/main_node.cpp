#include "multi_node_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_node_control");
    ros::NodeHandle nh;

    MultiNodeController controller(nh);

    ros::Rate rate(10); // 10 Hz control loop
    while (ros::ok()) {
        ros::spinOnce();
        controller.controlLoop();
        rate.sleep();
    }

    return 0;
}
