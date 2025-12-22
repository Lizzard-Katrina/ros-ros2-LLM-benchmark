#include "pointcloud_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_control_node");
    ros::NodeHandle nh;

    PointCloudController controller(nh);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        controller.controlLoop();
        rate.sleep();
    }

    return 0;
}
