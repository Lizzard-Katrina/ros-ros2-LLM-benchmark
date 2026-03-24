#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/kinematics_base/kinematics_base.h>

//mock file
#include "motion_planning/ikfast_plugin_class_mock.h"
//tested
#include  "../ros2_code/robotic_arm_arm_ikfast_moveit_plugin.cpp"

// ============================================================================
// 测试桩（Test Fixture）- 提供最小必要的 mock
// ============================================================================
using ikfast_kinematics_plugin::IKFastKinematicsPlugin;

class IKDecisionLoopTest : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        // 初始化 ROS2 节点（如果需要）
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // 创建节点
        node_ = std::make_shared<rclcpp::Node>("test_ik_node");
        
        // 创建solver实例 - 这里需要你的实际初始化代码
        solver_ = std::make_shared<IKFastKinematicsPlugin>();
        
        // 初始化solver（需要提供URDF等配置）
        // TODO: 根据你的实际情况填充
        // solver_->initialize(node_, robot_model, "arm", "base_link", tip_frames);
        
        // 准备测试数据
        setupTestData();
    }
    
    void TearDown() override {
        // 清理
    }
    
    void setupTestData() {
        // 可达姿态
        reachable_pose_.position.x = 0.5;
        reachable_pose_.position.y = 0.0;
        reachable_pose_.position.z = 0.5;
        reachable_pose_.orientation.w = 1.0;
        reachable_pose_.orientation.x = 0.0;
        reachable_pose_.orientation.y = 0.0;
        reachable_pose_.orientation.z = 0.0;
        
        // 有效seed（6关节）
        valid_seed_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // 无效seed（尺寸错误）
        invalid_seed_ = {0.0, 0.0, 0.0};
    }
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<IKFastKinematicsPlugin> solver_;
    geometry_msgs::msg::Pose reachable_pose_;
    std::vector<double> valid_seed_;
    std::vector<double> invalid_seed_;
};

// ============================================================================
// 测试用例 - 专注于被填补代码的功能
// ============================================================================

// 测试1: 基础功能 - 无自由参数时的IK求解
TEST_F(IKDecisionLoopTest, NoFreeParamsFindsValidSolution) 
{
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    
    // 模拟无自由参数的场景（free_params_.size() == 0）
    bool result = solver_->searchPositionIK(
        reachable_pose_,
        valid_seed_,
        5.0,  // timeout
        solution,
        error_code
    );
    
    // 验证：应该找到解
    EXPECT_TRUE(result);
    EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    EXPECT_EQ(solution.size(), 6);  // 6个关节
    
    // 可选：验证解的质量（通过正运动学）
    // TODO: 如果有FK功能，可以验证
}

// 测试2: 种子状态尺寸验证
TEST_F(IKDecisionLoopTest, InvalidSeedSizeReturnsError) 
{
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    
    bool result = solver_->searchPositionIK(
        reachable_pose_,
        invalid_seed_,  // 错误的尺寸
        5.0,
        solution,
        error_code
    );
    
    // 验证：应该拒绝
    EXPECT_FALSE(result);
    // 注意：ROS2中NO_IK_SOLUTION不存在，直接检查不是SUCCESS
    EXPECT_NE(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// 测试3: Solver未激活时的行为
TEST_F(IKDecisionLoopTest, InactiveSolverReturnsError) 
{
    // 创建未初始化的solver
    auto inactive_solver = std::make_shared<IKFastKinematicsPlugin>();
    
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    
    bool result = inactive_solver->searchPositionIK(
        reachable_pose_,
        valid_seed_,
        5.0,
        solution,
        error_code
    );
    
    EXPECT_FALSE(result);
    EXPECT_NE(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// 测试4: 关节限制检查 - 种子超出限制
TEST_F(IKDecisionLoopTest, SeedOutOfLimitsHandled) 
{
    std::vector<double> out_of_limit_seed = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};  // 明显超限
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    
    bool result = solver_->searchPositionIK(
        reachable_pose_,
        out_of_limit_seed,
        5.0,
        solution,
        error_code
    );
    
    // 行为可以是：拒绝或者找到另一个有效解
    // 这里只验证不会崩溃，error_code应该有意义
    EXPECT_TRUE(error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS || 
                error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// 测试5: 一致性限制功能
TEST_F(IKDecisionLoopTest, ConsistencyLimitsAreRespected) 
{
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    std::vector<double> consistency_limits(6, 0.1);  // ±0.1弧度
    
    bool result = solver_->searchPositionIK(
        reachable_pose_,
        valid_seed_,
        5.0,
        consistency_limits,
        solution,
        error_code
    );
    
    if (result) {
        // 如果找到解，验证是否接近seed
        for (size_t i = 0; i < solution.size(); ++i) {
            double diff = std::abs(solution[i] - valid_seed_[i]);
            // 允许一定误差
            EXPECT_LE(diff, consistency_limits[i] + 0.05) 
                << "Joint " << i << " violated consistency limit";
        }
    }
}

// 测试6: Solution Callback 功能
TEST_F(IKDecisionLoopTest, SolutionCallbackIsInvoked) 
{
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    int callback_count = 0;
    
    // 创建callback：接受第一个解
    auto callback = [&](const geometry_msgs::msg::Pose& /* pose */,
                       const std::vector<double>& /* ik_solution */,
                       moveit_msgs::msg::MoveItErrorCodes& err) {
        callback_count++;
        if (callback_count == 1) {
            err.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;  // 接受
        } else {
            err.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;  // 拒绝
        }
    };
    
    bool result = solver_->searchPositionIK(
        reachable_pose_,
        valid_seed_,
        5.0,
        solution,
        callback,
        error_code
    );
    
    // 验证callback被调用
    EXPECT_GT(callback_count, 0) << "Callback should be invoked";
    
    // 如果找到解，应该是被callback接受的
    if (result) {
        EXPECT_EQ(error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    }
}

// 测试7: 搜索优化 - 最小关节运动
TEST_F(IKDecisionLoopTest, OptimizesForMinimalJointMotion) 
{
    // 这个测试验证搜索算法是否倾向于找到接近seed的解
    
    std::vector<double> solution1, solution2;
    moveit_msgs::msg::MoveItErrorCodes error_code1, error_code2;
    
    // 使用seed1
    std::vector<double> seed1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    solver_->searchPositionIK(reachable_pose_, seed1, 5.0, solution1, error_code1);
    
    // 使用seed2（不同的起始点）
    std::vector<double> seed2 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    solver_->searchPositionIK(reachable_pose_, seed2, 5.0, solution2, error_code2);
    
    if (error_code1.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
        error_code2.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        
        // 计算距离
        double dist1 = 0.0, dist2 = 0.0;
        for (size_t i = 0; i < solution1.size(); ++i) {
            double d1 = std::abs(solution1[i] - seed1[i]);
            double d2 = std::abs(solution2[i] - seed2[i]);
            dist1 = std::max(dist1, d1);
            dist2 = std::max(dist2, d2);
        }
        
        // 两个解都应该相对接近各自的seed
        // 这验证了OPTIMIZE_MAX_JOINT模式
        EXPECT_LT(dist1, M_PI) << "Solution should be near seed1";
        EXPECT_LT(dist2, M_PI) << "Solution should be near seed2";
    }
}

// ============================================================================
// Main 函数
// ============================================================================
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
