#pragma once

#include <vector>

namespace geometry_msgs {
namespace msg {
struct Pose {};
}
}

namespace moveit_msgs {
namespace msg {
struct MoveItErrorCodes {
  int val = 0;
};
}
}

namespace kinematics {
struct KinematicsQueryOptions {};
}

class IKFastKinematicsPlugin
{
public:
  bool active_ = true;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options
  );
};
