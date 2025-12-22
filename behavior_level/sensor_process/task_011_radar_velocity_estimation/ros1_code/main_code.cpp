#include "radar_velocity_estimator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_velocity_node");
    ros::NodeHandle nh;

    RadarVelocityEstimator estimator(nh);

    ros::spin();
    return 0;
}
