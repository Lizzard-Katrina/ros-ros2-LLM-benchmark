# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
/*********************************************************************
 * (Based on MoveIt Pick and Place tutorial source)
 * https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
 *********************************************************************/

#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <trajectory_msgs/JointTrajectory.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // TODO(task_002):
  // Create and add collision objects representing:
  //  - table1 (support surface for picking)
  //  - table2 (support surface for placing)
  //  - object (the grasp target)
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // TODO(task_002):
  // Define one or more grasp configurations for the target object.
  // A grasp should include:
  //   - the desired grasp pose
  //   - approach and retreat motions
  //   - gripper posture before and during grasp
  //
  // Then command the robot to pick the target object.
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
// TODO(task_002):
  // Define one or more placement configurations for the object.
  // A placement should include:
  //   - the desired place pose
  //   - approach and retreat motions
  //   - gripper posture after placing
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  move_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);
  ros::WallDuration(1.0).sleep();

  pick(move_group);
  ros::WallDuration(1.0).sleep();

  place(move_group);

  ros::waitForShutdown();
  return 0;
}

----------------------------


# LLM Response

```cpp
/*********************************************************************
 * (Based on MoveIt Pick and Place tutorial source)
 * https://docs.ros.org/en/foxy/Tutorials/Moveit2-Tutorials.html
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Messages
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = rclcpp::Duration(0.5);
}

void closedGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = rclcpp::Duration(0.5);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Create and add collision objects representing:
  //  - table1 (support surface for picking)
  //  - table2 (support surface for placing)
  //  - object (the grasp target)
  moveit_msgs::msg::CollisionObject table1;
  table1.header.frame_id = "world";
  table1.id = "table1";
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  table1.pose = pose;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.1;
  table1.primitives.push_back(primitive);
  planning_scene_interface.addCollisionObject(table1);

  moveit_msgs::msg::CollisionObject table2;
  table2.header.frame_id = "world";
  table2.id = "table2";
  pose.position.x = -0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  table2.pose = pose;
  table2.primitives.push_back(primitive);
  planning_scene_interface.addCollisionObject(table2);

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "object";
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  object.pose = pose;
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;
  object.primitives.push_back(primitive);
  planning_scene_interface.addCollisionObject(object);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Define one or more grasp configurations for the target object.
  // A grasp should include:
  //   - the desired grasp pose
  //   - approach and retreat motions
  //   - gripper posture before and during grasp
  moveit_msgs::msg::Grasp grasp;
  grasp.grasp_pose.header.frame_id = "world";
  grasp.grasp_pose.pose.position.x = 0.0;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.5;
  grasp.grasp_pose.pose.orientation.w = 1.0;
  grasp.pre_grasp_posture.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(2);
  grasp.pre_grasp_posture.points[0].positions[0] = 0.04;
  grasp.pre_grasp_posture.points[0].positions[1] = 0.04;
  grasp.pre_grasp_posture.points[0].time_from_start = rclcpp::Duration(0.5);
  grasp.grasp_posture.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(2);
  grasp.grasp_posture.points[0].positions[0] = 0.00;
  grasp.grasp_posture.points[0].positions[1] = 0.00;
  grasp.grasp_posture.points[0].time_from_start = rclcpp::Duration(0.5);
  move_group.pick("object", grasp);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Define one or more placement configurations for the object.
  // A placement should include:
  //   - the desired place pose
  //   - approach and retreat motions
  //   - gripper posture after placing
  moveit_msgs::msg::PlaceLocation place_location;
  place_location.place_pose.header.frame_id = "world";
  place_location.place_pose.pose.position.x = -0.5;
  place_location.place_pose.pose.position.y = 0.0;
  place_location.place_pose.pose.position.z = 0.5;
  place_location.place_pose.pose.orientation.w = 1.0;
  place_location.post_place_posture.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
  place_location.post_place_posture.points.resize(1);
  place_location.post_place_posture.points[0].positions.resize(2);
  place_location.post_place_posture.points[0].positions[0] = 0.04;
  place_location.post_place_posture.points[0].positions[1] = 0.04;
  place_location.post_place_posture.points[0].time_from_start = rclcpp::Duration(0.5);
  move_group.place("object", place_location);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_arm_pick_place");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface(node);
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  move_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);
  rclcpp::sleep_for(rclcpp::Duration(1.0));

  pick(move_group);
  rclcpp::sleep_for(rclcpp::Duration(1.0));

  place(move_group);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```