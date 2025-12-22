#include "blob_tracker.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle nh;

    BlobTracker tracker(nh);

    ros::Rate rate(20); // 20 Hz control loop
    while (ros::ok()) {
        ros::spinOnce();
        tracker.controlLoop();
        rate.sleep();
    }

    return 0;
}
