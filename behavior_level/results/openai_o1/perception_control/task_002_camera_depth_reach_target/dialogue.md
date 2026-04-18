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

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <chrono>

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
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
}

void closedGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0;
  posture.points[0].positions[1] = 0.0;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // TODO(task_002):
  moveit_msgs::msg::CollisionObject table1, table2, object_to_pick;

  table1.id = "table1";
  table1.header.frame_id = "panda_link0";
  shape_msgs::msg::SolidPrimitive table1_primitive;
  table1_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  table1_primitive.dimensions.resize(3);
  table1_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
  table1_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.7;
  table1_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.5;
  table1_pose.position.y = 0.0;
  table1_pose.position.z = 0.2;
  table1.primitives.push_back(table1_primitive);
  table1.primitive_poses.push_back(table1_pose);
  table1.operation = table1.ADD;

  table2.id = "table2";
  table2.header.frame_id = "panda_link0";
  shape_msgs::msg::SolidPrimitive table2_primitive;
  table2_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  table2_primitive.dimensions.resize(3);
  table2_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
  table2_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.7;
  table2_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.5;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = 0.2;
  table2.primitives.push_back(table2_primitive);
  table2.primitive_poses.push_back(table2_pose);
  table2.operation = table2.ADD;

  object_to_pick.id = "object";
  object_to_pick.header.frame_id = "panda_link0";
  shape_msgs::msg::SolidPrimitive object_primitive;
  object_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object_primitive.dimensions.resize(2);
  object_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.2;
  object_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.5;
  object_pose.position.y = 0.0;
  object_pose.position.z = 0.5;
  object_to_pick.primitives.push_back(object_primitive);
  object_to_pick.primitive_poses.push_back(object_pose);
  object_to_pick.operation = object_to_pick.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(table1);
  collision_objects.push_back(table2);
  collision_objects.push_back(object_to_pick);

  planning_scene_interface.addCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // TODO(task_002):
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, M_PI / 2, 0.0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.5;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.45;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // TODO(task_002):
  std::vector<moveit_msgs::msg::PlaceLocation> place_locations;
  place_locations.resize(1);

  place_locations[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, M_PI / 2, 0.0);
  place_locations[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_locations[0].place_pose.pose.position.x = -0.5;
  place_locations[0].place_pose.pose.position.y = 0.0;
  place_locations[0].place_pose.pose.position.z = 0.45;

  place_locations[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_locations[0].pre_place_approach.direction.vector.z = -1.0;
  place_locations[0].pre_place_approach.min_distance = 0.095;
  place_locations[0].pre_place_approach.desired_distance = 0.115;

  place_locations[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_locations[0].post_place_retreat.direction.vector.y = -1.0;
  place_locations[0].post_place_retreat.min_distance = 0.1;
  place_locations[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_locations[0].post_place_posture);

  move_group.place("object", place_locations);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_arm_pick_place");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::sleep_for(std::chrono::seconds(1));

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  move_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);
  rclcpp::sleep_for(std::chrono::seconds(1));

  pick(move_group);
  rclcpp::sleep_for(std::chrono::seconds(1));

  place(move_group);

  rclcpp::shutdown();
  return 0;
}