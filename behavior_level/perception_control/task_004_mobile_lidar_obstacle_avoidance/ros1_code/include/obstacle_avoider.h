#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class ObstacleAvoider {
public:
    ObstacleAvoider(ros::NodeHandle& nh);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void controlLoop();

private:
    ros::Subscriber lidar_sub_;
    ros::Publisher cmd_pub_;

    double min_distance_;
    double linear_speed_;
    double angular_speed_;
    bool obstacle_detected_;
};

#endif
