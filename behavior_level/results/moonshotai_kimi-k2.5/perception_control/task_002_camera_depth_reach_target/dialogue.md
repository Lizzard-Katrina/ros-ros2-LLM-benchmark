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
 * https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
  
  moveit_msgs::msg::CollisionObject table1;
  table1.id = "table1";
  table1.header.frame_id = "panda_link0";
  
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.4;
  primitive.dimensions[2] = 0.4;
  
  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.5;
  table1_pose.position.y = 0.0;
  table1_pose.position.z = 0.2;
  
  table1.primitives.push_back(primitive);
  table1.primitive_poses.push_back(table1_pose);
  table1.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  moveit_msgs::msg::CollisionObject table2;
  table2.id = "table2";
  table2.header.frame_id = "panda_link0";
  
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = 0.0;
  table2_pose.position.y = 0.5;
  table2_pose.position.z = 0.2;
  
  table2.primitives.push_back(primitive);
  table2.primitive_poses.push_back(table2_pose);
  table2.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "panda_link0";
  
  shape_msgs::msg::SolidPrimitive object_primitive;
  object_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  object_primitive.dimensions.resize(3);
  object_primitive.dimensions[0] = 0.02;
  object_primitive.dimensions[1] = 0.02;
  object_primitive.dimensions[2] = 0.2;
  
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.5;
  object_pose.position.y = 0.0;
  object_pose.position.z = 0.5;
  
  object.primitives.push_back(object_primitive);
  object.primitive_poses.push_back(object_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  collision_objects.push_back(table1);
  collision_objects.push_back(table2);
  collision_objects.push_back(object);
  
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::Grasp> grasps;
  moveit_msgs::msg::Grasp grasp;
  
  grasp.grasp_pose.header.frame_id = "panda_link0";
  grasp.grasp_pose.pose.orientation.x = 0;
  grasp.grasp_pose.pose.orientation.y = 0;
  grasp.grasp_pose.pose.orientation.z = 0;
  grasp.grasp_pose.pose.orientation.w = 1;
  grasp.grasp_pose.pose.position.x = 0.5;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.5;
  
  grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasp.pre_grasp_approach.direction.vector.x = 0.0;
  grasp.pre_grasp_approach.direction.vector.y = 0.0;
  grasp.pre_grasp_approach.direction.vector.z = 1.0;
  grasp.pre_grasp_approach.min_distance = 0.095;
  grasp.pre_grasp_approach.desired_distance = 0.115;
  
  grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasp.post_grasp_retreat.direction.vector.x = 0.0;
  grasp.post_grasp_retreat.direction.vector.y = 0.0;
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.1;
  grasp.post_grasp_retreat.desired_distance = 0.25;
  
  openGripper(grasp.pre_grasp_posture);
  closedGripper(grasp.grasp_posture);
  
  grasps.push_back(grasp);
  
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::PlaceLocation> place_locations;
  moveit_msgs::msg::PlaceLocation place_location;
  
  place_location.place_pose.header.frame_id = "panda_link0";
  place_location.place_pose.pose.orientation.x = 0;
  place_location.place_pose.pose.orientation.y = 0;
  place_location.place_pose.pose.orientation.z = 0;
  place_location.place_pose.pose.orientation.w = 1;
  place_location.place_pose.pose.position.x = 0.0;
  place_location.place_pose.pose.position.y = 0.5;
  place_location.place_pose.pose.position.z = 0.5;
  
  place_location.pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location.pre_place_approach.direction.vector.x = 0.0;
  place_location.pre_place_approach.direction.vector.y = 0.0;
  place_location.pre_place_approach.direction.vector.z = -1.0;
  place_location.pre_place_approach.min_distance = 0.095;
  place_location.pre_place_approach.desired_distance = 0.115;
  
  place_location.post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location.post_place_retreat.direction.vector.x = 0.0;
  place_location.post_place_retreat.direction.vector.y = 0.0;
  place_location.post_place_retreat.direction.vector.z = -1.0;
  place_location.post_place_retreat.min_distance = 0.1;
  place_location.post_place_retreat.desired_distance = 0.25;
  
  openGripper(place_location.post_place_posture);
  
  place_locations.push_back(place_location);
  
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

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface(node);
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
```