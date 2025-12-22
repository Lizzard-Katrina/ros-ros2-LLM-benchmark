#include "nav_system.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_system");
    ros::NodeHandle nh;

    NavSystem nav(nh);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        nav.fuseSensors();
        nav.planPath();
        nav.controlMotion();

        rate.sleep();
    }
    return 0;
}
