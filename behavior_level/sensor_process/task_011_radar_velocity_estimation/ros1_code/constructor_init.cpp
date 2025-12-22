#include "radar_velocity_estimator.h"

RadarVelocityEstimator::RadarVelocityEstimator(ros::NodeHandle& nh) {
    radar_sub_ = nh.subscribe("/radar/scan", 100, &RadarVelocityEstimator::radarCallback, this);
    velocity_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/radar_velocity", 10);

    // TODO: Initialize radar velocity estimation algorithm (skeleton)
    // --------------------------
    // 1. Initialize radar data buffers
    // 2. Load algorithm parameters
    // 3. Initialize FFT / Doppler processing modules
    // END
}
