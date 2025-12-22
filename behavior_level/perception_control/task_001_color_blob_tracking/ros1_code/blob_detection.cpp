#include "blob_tracker.h"

void BlobTracker::cameraCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
    cv::Mat cv_image;
    try {
        cv_image = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // TODO: Detect color blob in the image and update blob_center_ & blob_detected_
    // --------------------------
    // - Convert BGR → HSV
    // - Threshold using target_color_lower_ and target_color_upper_
    // - Find contours and select largest blob
    // - Update blob_center_ and blob_detected_
    // END
}
