#pragma once

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace kinematics {
struct KinematicsQueryOptions {};
}

namespace moveit_msgs {
struct MoveItErrorCodes {
  int val;
  static const int SUCCESS = 1;
  static const int NO_IK_SOLUTION = -1;
};
}

namespace geometry_msgs {
struct Pose {};
}

namespace ikfast_kinematics_plugin {

class IKFastKinematicsPlugin
{
public:
  bool active_{false};
};

}  // namespace ikfast_kinematics_plugin
