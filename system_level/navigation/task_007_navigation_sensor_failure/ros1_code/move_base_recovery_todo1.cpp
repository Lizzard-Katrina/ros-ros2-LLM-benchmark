#include <move_base/move_base.h>

void MoveBase::executeCycle(const geometry_msgs::PoseStamped& goal)
{
  if (new_global_plan_) {
    state_ = CONTROLLING;
  }

  switch (state_) {

    case CONTROLLING:
      if (!tc_->computeVelocityCommands(cmd_vel)) {
         state_ = CLEARING;
         recovery_index_ = 0;
       }
      break;

    case CLEARING:
      // TODO: execute recovery behaviors sequentially
      break;

    default:
      break;
  }

  // END
}
