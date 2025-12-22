#include "radar_velocity_estimator.h"
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>

void RadarVelocityEstimator::radarCallback(const astuff_sensor_msgs::RadarScan::ConstPtr& radar_msg) {
    // Append radar data to buffer
    radarBuffer_.push_back(radar_msg->ranges);

    // TODO: Compute radial velocities from radar signals
    // --------------------------
    // - Perform Doppler processing / FFT
    // - Estimate radial velocity for each detected object
    // - Update radialVelocities_ vector
    // END

    // Publish estimated velocities
    std_msgs::Float32MultiArray velMsg;
    velMsg.data = radialVelocities_;
    velocity_pub_.publish(velMsg);
}
