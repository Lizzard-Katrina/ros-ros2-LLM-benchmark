#ifndef SIM_NAV_H
#define SIM_NAV_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class SimNav {
public:
    SimNav(ros::NodeHandle& nh);

    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void processPerception();
    void computeControl();

private:
    ros::Subscriber camera_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher  cmd_pub_;

    sensor_msgs::Image latest_image_;
    sensor_msgs::LaserScan latest_lidar_;

    bool image_received_;
    bool lidar_received_;
};

#endif
