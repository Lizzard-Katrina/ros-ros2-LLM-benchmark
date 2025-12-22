#include "arm_controller.h"

ArmController::ArmController(ros::NodeHandle& nh)
    : move_group_("arm") {
    cloud_sub_ = nh.subscribe("/camera/depth/points", 10, &ArmController::pointCloudCallback, this);

    // TODO: Initialize MoveIt interface and parameters
    // --------------------------
    // END
}
