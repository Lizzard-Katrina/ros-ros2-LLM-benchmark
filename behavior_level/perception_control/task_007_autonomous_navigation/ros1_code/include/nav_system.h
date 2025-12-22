#ifndef NAV_SYSTEM_H
#define NAV_SYSTEM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

class NavSystem {
public:
    NavSystem(ros::NodeHandle& nh);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);

    void fuseSensors();
    void planPath();
    void controlMotion();

private:
    ros::Subscriber lidar_sub_;
    ros::Subscriber camera_sub_;
    ros::Publisher  cmd_pub_;
    ros::Publisher  path_pub_;

    sensor_msgs::LaserScan latest_lidar_;
    sensor_msgs::Image latest_image_;

    bool lidar_received_;
    bool camera_received_;

    nav_msgs::Path planned_path_;
};

#endif
