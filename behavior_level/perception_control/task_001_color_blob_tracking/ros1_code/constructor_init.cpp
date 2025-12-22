#include "blob_tracker.h"

BlobTracker::BlobTracker(ros::NodeHandle& nh) {
    camera_sub_ = nh.subscribe("/camera/image_raw", 10, &BlobTracker::cameraCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Initialize blob tracking parameters
    // --------------------------
    // END
}
