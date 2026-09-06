/*********************************************************************
 * (Based on MoveIt Pick and Place tutorial source)
 * ROS2 / MoveIt2 translation
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Table 1 (support surface for picking)
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

  // Table 2 (support surface for placing)
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

  // Object to grasp
  collision_objects[2].id = "object";
  collision_objects[2].header.frame_id = "panda_link0";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0.0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  // Grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Pre-grasp posture (open gripper)
  openGripper(grasps[0].pre_grasp_posture);

  // Grasp posture (close gripper)
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::PlaceLocation> place_location;
  place_location.resize(1);

  // Place pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = 0.0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Post-place retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Post-place posture (open gripper to release)
  openGripper(place_location[0].post_place_posture);

  move_group.setSupportSurfaceName("table2");
  move_group.place("object", place_location);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_arm_pick_place");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

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
  spinner.join();
  return 0;
}