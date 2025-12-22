#ifndef SENSOR_FUSION_VISUALIZER_H
#define SENSOR_FUSION_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>

class SensorFusionVisualizer {
public:
    SensorFusionVisualizer(ros::NodeHandle& nh);

    void cameraCallback(const sensor_msgs::Image::ConstPtr& img_msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);

private:
    ros::Subscriber camera_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher fused_marker_pub_;

    // Sensor buffers
    sensor_msgs::Image latest_image_;
    sensor_msgs::PointCloud2 latest_cloud_;

    // Fusion / visualization parameters
    double voxel_size_;
    bool enable_projection_;
};

#endif
