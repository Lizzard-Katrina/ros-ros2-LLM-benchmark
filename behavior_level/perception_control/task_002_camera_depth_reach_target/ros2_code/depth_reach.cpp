/*********************************************************************
 * (Based on MoveIt Pick and Place tutorial source)
 * https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

  // Table1
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive table1_primitive;
  table1_primitive.type = table1_primitive.BOX;
  table1_primitive.dimensions = {0.5, 1.5, 0.35};

  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.5;
  table1_pose.position.y = 0.0;
  table1_pose.position.z = 0.175;

  collision_objects[0].primitives.push_back(table1_primitive);
  collision_objects[0].primitive_poses.push_back(table1_pose);
  collision_objects[0].operation = collision_objects[0].ADD;

  // Table2
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive table2_primitive;
  table2_primitive.type = table2_primitive.BOX;
  table2_primitive.dimensions = {0.5, 0.5, 0.35};

  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = 0.0;
  table2_pose.position.y = -0.7;
  table2_pose.position.z = 0.175;

  collision_objects[1].primitives.push_back(table2_primitive);
  collision_objects[1].primitive_poses.push_back(table2_pose);
  collision_objects[1].operation = collision_objects[1].ADD;

  // Object to pick
  collision_objects[2].id = "object";
  collision_objects[2].header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive object_primitive;
  object_primitive.type = object_primitive.BOX;
  object_primitive.dimensions = {0.05, 0.05, 0.2};

  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.5;
  object_pose.position.y = 0.0;
  object_pose.position.z = 0.6;

  collision_objects[2].primitives.push_back(object_primitive);
  collision_objects[2].primitive_poses.push_back(object_pose);
  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, 0, -M_PI / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.5;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.6;

  // Setting pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Open gripper before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Closed gripper during grasp
  closedGripper(grasps[0].grasp_posture);

  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::PlaceLocation> place_locations;
  place_locations.resize(1);

  // Setting place pose
  place_locations[0].place_pose.header.frame_id = "panda_link0";
  place_locations[0].place_pose.pose.orientation.w = 1.0;
  place_locations[0].place_pose.pose.position.x = 0.0;
  place_locations[0].place_pose.pose.position.y = -0.7;
  place_locations[0].place_pose.pose.position.z = 0.6;

  // Setting pre-place approach
  place_locations[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_locations[0].pre_place_approach.direction.vector.z = -1.0;
  place_locations[0].pre_place_approach.min_distance = 0.095;
  place_locations[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-place retreat
  place_locations[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_locations[0].post_place_retreat.direction.vector.y = -1.0;
  place_locations[0].post_place_retreat.min_distance = 0.1;
  place_locations[0].post_place_retreat.desired_distance = 0.25;

  // Open gripper after placing
  openGripper(place_locations[0].post_place_posture);

  move_group.place("object", place_locations);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_arm_pick_place");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Create MoveGroupInterface and PlanningSceneInterface with node
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  move_group.setPlanningTime(45.0);

  // Run async spinner in separate thread
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::sleep_for(std::chrono::seconds(1));

  addCollisionObjects(planning_scene_interface);
  rclcpp::sleep_for(std::chrono::seconds(1));

  pick(move_group);
  rclcpp::sleep_for(std::chrono::seconds(1));

  place(move_group);

  rclcpp::shutdown();
  return 0;
}