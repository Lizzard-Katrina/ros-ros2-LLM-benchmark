#pragma once
#include <moveit_msgs/msg/move_it_error_codes.hpp>
namespace moveit_msgs
{

struct MoveItErrorCodes
{
  enum
  {
    SUCCESS = 1,
    FAILURE = 99999,
    NO_IK_SOLUTION = -31,
    PLANNING_FAILED = -1
  };

  int val = FAILURE;
};

}  // namespace moveit_msgs
