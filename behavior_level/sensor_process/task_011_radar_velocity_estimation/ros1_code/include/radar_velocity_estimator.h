#ifndef RADAR_VELOCITY_ESTIMATOR_H
#define RADAR_VELOCITY_ESTIMATOR_H

#include <ros/ros.h>
#include <astuff_sensor_msgs/RadarScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <Eigen/Dense>

class RadarVelocityEstimator {
public:
    RadarVelocityEstimator(ros::NodeHandle& nh);

    void radarCallback(const astuff_sensor_msgs::RadarScan::ConstPtr& radar_msg);

private:
    ros::Subscriber radar_sub_;
    ros::Publisher velocity_pub_;

    // Radar data buffer
    std::vector<std::vector<float>> radarBuffer_;
    std::vector<float> radialVelocities_;

    // Algorithm parameters
    int numBeams_;
    double sampleRate_;
    double maxRange_;
};

#endif
