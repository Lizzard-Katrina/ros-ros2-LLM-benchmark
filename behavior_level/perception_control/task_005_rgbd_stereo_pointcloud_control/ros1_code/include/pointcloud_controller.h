#ifndef POINTCLOUD_CONTROLLER_H
#define POINTCLOUD_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudController {
public:
    PointCloudController(ros::NodeHandle& nh);

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void controlLoop();

private:
    ros::Subscriber cloud_sub_;
    ros::Publisher cmd_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    bool obstacle_detected_;
    double linear_speed_;
    double angular_speed_;
};

#endif
