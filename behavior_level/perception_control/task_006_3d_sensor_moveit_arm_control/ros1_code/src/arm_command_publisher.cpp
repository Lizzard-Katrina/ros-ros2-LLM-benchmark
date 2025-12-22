#include "arm_controller.h"

void ArmController::executeMotion() {
    if (!new_target_available_) return;

    // TODO: Execute planned motion
    // --------------------------
    // - Send trajectory to arm controller
    // - Reset new_target_available_ if needed
    // END
}
