#include "arm_reacher.h"

ArmReacher::ArmReacher(ros::NodeHandle& nh) {
    depth_camera_sub_ = nh.subscribe("/camera/depth/image_raw", 10, &ArmReacher::depthCameraCallback, this);

    // TODO: Initialize MoveIt planning parameters
    // --------------------------
    // END
}
