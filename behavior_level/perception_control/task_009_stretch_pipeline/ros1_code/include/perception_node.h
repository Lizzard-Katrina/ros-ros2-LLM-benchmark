#ifndef STRETCH_PIPELINE_PERCEPTION_NODE_H
#define STRETCH_PIPELINE_PERCEPTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

namespace stretch_pipeline {

struct ObjectDetection {
    std::string id;
    geometry_msgs::Pose pose;
    // add more fields as needed
};

class PerceptionNode {
public:
    PerceptionNode(ros::NodeHandle& nh);

    void colorCallback(const sensor_msgs::Image::ConstPtr& img);
    void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

private:
    ros::Subscriber color_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher  det_pub_;

    // Internal buffers / models
    // (Keep simple; LLM fills core detection logic)
};

} // namespace
#endif
