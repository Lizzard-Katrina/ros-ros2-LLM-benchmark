#ifndef BLOB_TRACKER_H
#define BLOB_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BlobTracker {
public:
    BlobTracker(ros::NodeHandle& nh);

    void cameraCallback(const sensor_msgs::Image::ConstPtr& img_msg);
    void controlLoop();

private:
    ros::Subscriber camera_sub_;
    ros::Publisher cmd_pub_;

    cv::Scalar target_color_lower_;
    cv::Scalar target_color_upper_;
    cv::Point2f blob_center_;
    bool blob_detected_;
    double max_speed_;
};

#endif
