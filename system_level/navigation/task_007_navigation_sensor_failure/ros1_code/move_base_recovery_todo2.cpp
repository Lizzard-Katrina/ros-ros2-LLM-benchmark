#include <move_base/move_base.h>

void MoveBase::executeCycle(const geometry_msgs::PoseStamped& goal)
{
  if (new_global_plan_) {
    state_ = CONTROLLING;
  }

  switch (state_) {

    case CONTROLLING:
      // TODO: decide what to do when controller fails
      break;

    case CLEARING:
       if (recovery_index_ < recovery_behaviors_.size()) {
         recovery_behaviors_[recovery_index_]->runBehavior();
         recovery_index_++;
       } else {
         state_ = ABORT;
       }
      break;

    default:
      break;
  }

  // END
}
