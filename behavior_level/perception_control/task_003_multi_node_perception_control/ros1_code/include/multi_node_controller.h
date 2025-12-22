#ifndef MULTI_NODE_CONTROLLER_H
#define MULTI_NODE_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class MultiNodeController {
public:
    MultiNodeController(ros::NodeHandle& nh);

    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void controlLoop();

private:
    ros::Subscriber sensor_sub_;
    ros::Publisher cmd_pub_;

    double processed_value_;
    bool data_ready_;
};

#endif
