#include "obstacle_avoider.h"

void ObstacleAvoider::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // TODO: Detect obstacles from LiDAR scan
    // --------------------------
    // - Check scan ranges
    // - Set obstacle_detected_ = true if any reading < min_distance_
    // - Optionally store distance/direction to closest obstacle
    // END
}
