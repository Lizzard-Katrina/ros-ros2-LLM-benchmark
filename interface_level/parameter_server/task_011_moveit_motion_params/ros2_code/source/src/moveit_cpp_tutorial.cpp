#include <rclcpp/rclcpp.h>
#include <memory>
#include <thread>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
  // ROS 2 initialization
  rclcpp::init(argc, argv);

  // Configure NodeOptions to allow MoveIt 2 YAML parameter loading
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);

  // Create the ROS 2 node with the configured options
  auto node = std::make_shared<rclcpp::Node>("moveit_cpp_tutorial", "", node_options);

  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string LOGNAME = "moveit_cpp_tutorial";

  // We need a background thread spinning the node so that MoveItCpp internal
  // monitors (PlanningSceneMonitor, CurrentStateMonitor) can receive updates
  // and the constructor does not deadlock.
  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });

  // Allow some time for the robot state to be received
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO_STREAM(node->get_logger(), "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Plan #1
  planning_components->setStartStateToCurrentState();

  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "panda_link0";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.28;
  target_pose1.pose.position.y = -0.2;
  target_pose1.pose.position.z = 0.5;
  planning_components->setGoal(target_pose1, "panda_link8");

  auto plan_solution1 = planning_components->plan();

  if (plan_solution1)
  {
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("panda_link8"), "start_pose");
    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory_, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Plan #2
  auto start_state = *(moveit_cpp_ptr->getCurrentState());
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.6;

  start_state.setFromIK(joint_model_group_ptr, start_pose);

  planning_components->setStartState(start_state);

  auto plan_solution2 = planning_components->plan();
  if (plan_solution2)
  {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution2.start_state_, robot_state);

    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishTrajectoryLine(plan_solution2.trajectory_, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Plan #3
  auto target_state = *robot_start_state;
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.55;
  target_pose2.position.y = -0.05;
  target_pose2.position.z = 0.8;

  target_state.setFromIK(joint_model_group_ptr, target_pose2);

  planning_components->setGoal(target_state);

  auto plan_solution3 = planning_components->plan();
  if (plan_solution3)
  {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution3.start_state_, robot_state);

    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose2, "target_pose");
    visual_tools.publishTrajectoryLine(plan_solution3.trajectory_, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Plan #4
  planning_components->setGoal("ready");

  auto plan_solution4 = planning_components->plan();
  if (plan_solution4)
  {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution4.start_state_, robot_state);

    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_link8"), "start_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("panda_link8"), "target_pose");
    visual_tools.publishTrajectoryLine(plan_solution4.trajectory_, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' to end the demo");

  RCLCPP_INFO_STREAM(node->get_logger(), "Shutting down.");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}