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

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

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
  posture.points[0].time_from_start = rclcpp::Duration(0, 500000000); // 0.5 seconds
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
  posture.points[0].time_from_start = rclcpp::Duration(0, 500000000); // 0.5 seconds
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Create collision objects
  moveit_msgs::msg::CollisionObject collision_object_table1;
  collision_object_table1.header.frame_id = "panda_link0";
  collision_object_table1.id = "table1";

  shape_msgs::msg::SolidPrimitive primitive_table1;
  primitive_table1.type = primitive_table1.BOX;
  primitive_table1.dimensions = {0.5, 1.5, 0.3}; // x, y, z

  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.5;
  table1_pose.position.y = 0.0;
  table1_pose.position.z = -0.15;

  collision_object_table1.primitives.push_back(primitive_table1);
  collision_object_table1.primitive_poses.push_back(table1_pose);
  collision_object_table1.operation = collision_object_table1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_table2;
  collision_object_table2.header.frame_id = "panda_link0";
  collision_object_table2.id = "table2";

  shape_msgs::msg::SolidPrimitive primitive_table2;
  primitive_table2.type = primitive_table2.BOX;
  primitive_table2.dimensions = {0.5, 1.5, 0.3}; // x, y, z

  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.5;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = -0.15;

  collision_object_table2.primitives.push_back(primitive_table2);
  collision_object_table2.primitive_poses.push_back(table2_pose);
  collision_object_table2.operation = collision_object_table2.ADD;

  moveit_msgs::msg::CollisionObject collision_object_object;
  collision_object_object.header.frame_id = "panda_link0";
  collision_object_object.id = "object";

  shape_msgs::msg::SolidPrimitive primitive_object;
  primitive_object.type = primitive_object.BOX;
  primitive_object.dimensions = {0.07, 0.07, 0.07}; // x, y, z

  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.5;
  object_pose.position.y = 0.0;
  object_pose.position.z = 0.185;

  collision_object_object.primitives.push_back(primitive_object);
  collision_object_object.primitive_poses.push_back(object_pose);
  collision_object_object.operation = collision_object_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_table1);
  collision_objects.push_back(collision_object_table2);
  collision_objects.push_back(collision_object_object);

  planning_scene_interface.add_collision_objects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.48;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.185;

  // Setting pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::msg::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = -0.48;
  place_location[0].place_pose.pose.position.y = 0.0;
  place_location[0].place_pose.pose.position.z = 0.185;

  // Setting pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-place retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing
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
```