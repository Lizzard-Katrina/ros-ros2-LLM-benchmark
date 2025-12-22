#include "sim_nav.h"
#include <geometry_msgs/Twist.h>

void SimNav::computeControl() {
    if (!lidar_received_) return;

    geometry_msgs::Twist cmd;

    // TODO: Compute virtual control commands from perception output
    // --------------------------------------------------------------
    // - If obstacle detected: turn / slow down
    // - If path is clear: move forward
    // - Apply simulation-safe speed limits
    // END

    cmd_pub_.publish(cmd);
}
