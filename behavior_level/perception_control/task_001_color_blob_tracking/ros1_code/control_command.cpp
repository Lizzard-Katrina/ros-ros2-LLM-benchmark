#include "blob_tracker.h"
#include <geometry_msgs/Twist.h>

void BlobTracker::controlLoop() {
    if (!blob_detected_) {
        // Stop the robot if no blob detected
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_pub_.publish(stop_msg);
        return;
    }

    // TODO: Convert blob_center_ into velocity command (cmd_vel)
    // --------------------------
    // - Compute error between image center and blob_center_
    // - Apply proportional controller for linear and angular velocity
    // - Publish cmd_vel
    // END
}
