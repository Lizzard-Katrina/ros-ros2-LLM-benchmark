#pragma once

namespace moveit_msgs
{
namespace msg
{

struct MoveItErrorCodes
{
  int val;

  enum
  {
    SUCCESS = 1,
    FAILURE = 99999,
    NO_IK_SOLUTION = -31
  };

  MoveItErrorCodes() : val(SUCCESS) {}
};

}  // namespace msg
}  // namespace moveit_msgs
