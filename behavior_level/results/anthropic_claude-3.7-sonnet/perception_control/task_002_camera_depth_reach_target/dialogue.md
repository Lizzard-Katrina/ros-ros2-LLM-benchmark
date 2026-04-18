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
  // Create vector to hold collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the object will be placed
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.05;

  // Define the pose of the table
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will place the object
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.5;
  collision_objects[1].primitives[0].dimensions[1] = 0.5;
  collision_objects[1].primitives[0].dimensions[2] = 0.05;

  // Define the pose of the table
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object to be grasped
  collision_objects[2].id = "object";
  collision_objects[2].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  // Define the pose of the object
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.325;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  // Add the objects to the planning scene
  planning_scene_interface.addCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create a vector of grasps to be attempted
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.5;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.325;

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

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1
  move_group.setSupportSurfaceName("table1");

  // Call pick to pick up the object
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create a vector of place locations to be attempted
  std::vector<moveit_msgs::msg::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.325;

  // Setting pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-place retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2
  move_group.setSupportSurfaceName("table2");

  // Call place to place the object
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