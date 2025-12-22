#include "nav_system.h"

void NavSystem::controlMotion() {
    if (planned_path_.poses.empty()) return;

    geometry_msgs::Twist cmd;

    // TODO: Convert planned path to velocity commands
    // --------------------------
    // - Track first few waypoints
    // - Compute angular error
    // - Compute linear velocity
    // - Apply speed limits
    // END

    cmd_pub_.publish(cmd);
}
