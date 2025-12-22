#include "pose_estimator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_sam_node");
    ros::NodeHandle nh;

    PoseEstimator estimator(nh);

    ros::spin();
    return 0;
}
