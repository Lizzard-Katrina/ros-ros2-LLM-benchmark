#include "stretch_pipeline/perception_node.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <stretch_msgs/ObjectDetections.h> // placeholder

using namespace stretch_pipeline;

PerceptionNode::PerceptionNode(ros::NodeHandle& nh) {
    color_sub_ = nh.subscribe("/camera/color/image_raw", 5, &PerceptionNode::colorCallback, this);
    depth_sub_ = nh.subscribe("/camera/depth/points", 5, &PerceptionNode::depthCallback, this);
    det_pub_   = nh.advertise<stretch_msgs::ObjectDetections>("/detected_objects", 10);
}

void PerceptionNode::colorCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
    cv::Mat color;
    try {
        color = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // TODO: Object detection logic (core)
    // -----------------------------------
    // - Preprocess image (resize / blur)
    // - Run detection (e.g., color thresholding or ML model)
    // - For each detection, compute 2D bbox and optionally use depth subscription to get 3D pose
    // - Fill stretch_msgs::ObjectDetections message and publish
    // END

    // Example: publish empty placeholder for now
    stretch_msgs::ObjectDetections dets;
    det_pub_.publish(dets);
}

void PerceptionNode::depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Depth point cloud could be used to lift 2D detections to 3D
    // This file keeps the depth subscriber logic intact; core fusion can be done in TODO above
}
