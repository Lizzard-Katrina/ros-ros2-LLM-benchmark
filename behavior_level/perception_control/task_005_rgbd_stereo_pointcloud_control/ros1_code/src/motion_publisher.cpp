#include "pointcloud_controller.h"
#include <geometry_msgs/Twist.h>

void PointCloudController::controlLoop() {
    geometry_msgs::Twist cmd;

    if (obstacle_detected_) {
        // TODO: Compute avoidance behavior
        // --------------------------
        // - Stop or turn robot
        // - Publish cmd to /cmd_vel
        // END
    } else {
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
        cmd_pub_.publish(cmd);
    }
}
