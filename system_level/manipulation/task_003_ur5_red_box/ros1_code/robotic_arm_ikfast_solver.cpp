#include "robotic_arm_ikfast_solver/robotic_arm_ikfast_solver.h"
#include <vector>

namespace robotic_arm
{

class RoboticArmIKFastSolver
{
public:
    RoboticArmIKFastSolver() {}
    ~RoboticArmIKFastSolver() {}

    bool getBestSolution(const double* eetrans,
                         const double* eerot,
                         const std::vector<int>& free_params,
                         std::vector<double>& solution);

private:
    // Internal solver member
    size_t num_joints_;
};

bool RoboticArmIKFastSolver::getBestSolution(const double* eetrans,
                                             const double* eerot,
                                             const std::vector<int>& free_params,
                                             std::vector<double>& solution)
{
    // TODO: Fill in the logic to select a feasible IK solution from candidate solutions
    // end: Fill-in ends

    return true;
}

} // namespace robotic_arm
