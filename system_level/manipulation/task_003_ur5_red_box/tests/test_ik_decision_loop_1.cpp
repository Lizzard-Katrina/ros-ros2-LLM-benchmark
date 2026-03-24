#include "motion_planning/ikfast_oracle_interface.h"
//#include "motion_planning/ikfast_plugin_class_mock.h"
#include <gtest/gtest.h>

class IKFastOracleTest : public ::testing::Test {
protected:
    IKFastKinematicsPlugin solver;

    void SetUp() override {
        solver.active_ = true;
        solver.joint_has_limits_vector_ = {true, true, true, true, true, true};
    }
};

// 1. Test inactive solver
TEST_F(IKFastOracleTest, InactiveSolverReturnsFalse) {
    solver.active_ = false;
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    geometry_msgs::msg::Pose pose;
    std::vector<double> seed(6, 0.0);

    bool ret = solver.searchPositionIK(pose, seed, solution, error_code, {});
    EXPECT_FALSE(ret);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

// 2. Test seed size mismatch
TEST_F(IKFastOracleTest, SeedSizeMismatch) {
    std::vector<double> seed(4, 0.0);  // 少于 num_joints_
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    geometry_msgs::msg::Pose pose;

    bool ret = solver.searchPositionIK(pose, seed, solution, error_code, {});
    EXPECT_FALSE(ret);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

// 3. Test joint limits violation
TEST_F(IKFastOracleTest, JointLimitViolation) {
    std::vector<double> seed = {2, 0, 0, 0, 0, 0}; // 第一关节超上限
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    geometry_msgs::msg::Pose pose;

    bool ret = solver.searchPositionIK(pose, seed, solution, error_code, {});
    EXPECT_FALSE(ret);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

// 4. Test valid solution exists
TEST_F(IKFastOracleTest, ValidSolution) {
    std::vector<double> seed(6, 0.0);
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    geometry_msgs::msg::Pose pose;

    bool ret = solver.searchPositionIK(pose, seed, solution, error_code, {});
    EXPECT_TRUE(ret);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
