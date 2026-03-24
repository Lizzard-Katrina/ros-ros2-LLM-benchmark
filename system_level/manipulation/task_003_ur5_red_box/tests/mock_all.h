#ifndef MOCK_ALL_H
#define MOCK_ALL_H

#include <vector>
#include <functional>

namespace moveit_msgs {
namespace msg {

// Mock of MoveItErrorCodes
struct MoveItErrorCodes {
    int val = 0;

    // Some common error codes (mock values)
    static const int SUCCESS = 1;
    static const int NO_IK_SOLUTION = 2;
    // add more if needed
};

}  // namespace msg
}  // namespace moveit_msgs

namespace kinematics {

// Mock KinematicsResult
struct KinematicsResult {
    int kinematic_error = 0;
};

// Mock KinematicsQueryOptions
struct KinematicsQueryOptions {
    int search_mode = 0;
    int discretization_method = 0;  // for sampleRedundantJoint
};

// Mock enum for DiscretizationMethods
struct DiscretizationMethods {
    static const int NO_DISCRETIZATION = 0;
};

// Mock enum for KinematicErrors
struct KinematicErrors {
    static const int SOLVER_NOT_ACTIVE = 1;
    static const int EMPTY_TIP_POSES = 2;
    static const int MULTIPLE_TIPS_NOT_SUPPORTED = 3;
    static const int IK_SEED_OUTSIDE_LIMITS = 4;
    static const int UNSUPORTED_DISCRETIZATION_REQUESTED = 5;
    static const int OK = 6;
    static const int NO_SOLUTION = 7;
};

}  // namespace kinematics

namespace KDL {

// Mock KDL::Frame
struct Frame {
    // empty for mock
};

}  // namespace KDL

// Mock IK types
template <typename T>
struct IkSolutionList {
    std::vector<std::vector<T>> solutions;
};

// Mock IKReal type
using IkReal = double;

// Mock IK callback function type
using IKCallbackFn = std::function<bool(const std::vector<double>&)>;

// Mock geometry_msgs::msg::Pose
namespace geometry_msgs {
namespace msg {

struct Pose {
    double position[3] = {0.0, 0.0, 0.0};
    double orientation[4] = {0.0, 0.0, 0.0, 1.0};
};

}  // namespace msg
}  // namespace geometry_msgs

// Mock tf2 namespace
namespace tf2 {

inline void fromMsg(const geometry_msgs::msg::Pose&, KDL::Frame&) {
    // empty mock
}

}  // namespace tf2

#endif  // MOCK_ALL_H
