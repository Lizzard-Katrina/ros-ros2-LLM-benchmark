#include "arm_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh;

    ArmController controller(nh);
    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();
        controller.planMotion();
        controller.executeMotion();
        rate.sleep();
    }

    return 0;
}
