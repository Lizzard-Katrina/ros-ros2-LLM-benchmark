/*********************************************************************
 * Stub executable built when moveit_ros_planning_interface is not
 * available.  It exercises the same ROS2 message types and rclcpp
 * APIs used by the real depth_reach.cpp, proving the package links
 * correctly against all message dependencies.
 *
 * It publishes CollisionObject messages on the "collision_object"
 * topic using transient-local durability so late-joining subscribers
 * can still receive them, and waits for at least one subscriber
 * before publishing.
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

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

std::vector<moveit_msgs::msg::CollisionObject> buildCollisionObjects()
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(3);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions = {0.2, 0.4, 0.4};
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[1].primitives[0].dimensions = {0.4, 0.2, 0.4};
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_objects[2].id = "object";
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[2].primitives[0].dimensions = {0.02, 0.02, 0.2};
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0.0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

  return collision_objects;
}

moveit_msgs::msg::Grasp buildGrasp()
{
  moveit_msgs::msg::Grasp grasp;

  grasp.grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasp.grasp_pose.pose.position.x = 0.415;
  grasp.grasp_pose.pose.position.y = 0.0;
  grasp.grasp_pose.pose.position.z = 0.5;

  grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasp.pre_grasp_approach.direction.vector.x = 1.0;
  grasp.pre_grasp_approach.min_distance = 0.095;
  grasp.pre_grasp_approach.desired_distance = 0.115;

  grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.1;
  grasp.post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasp.pre_grasp_posture);
  closedGripper(grasp.grasp_posture);

  return grasp;
}

moveit_msgs::msg::PlaceLocation buildPlaceLocation()
{
  moveit_msgs::msg::PlaceLocation pl;

  pl.place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);
  pl.place_pose.pose.orientation = tf2::toMsg(orientation);
  pl.place_pose.pose.position.x = 0.0;
  pl.place_pose.pose.position.y = 0.5;
  pl.place_pose.pose.position.z = 0.5;

  pl.pre_place_approach.direction.header.frame_id = "panda_link0";
  pl.pre_place_approach.direction.vector.z = -1.0;
  pl.pre_place_approach.min_distance = 0.095;
  pl.pre_place_approach.desired_distance = 0.115;

  pl.post_place_retreat.direction.header.frame_id = "panda_link0";
  pl.post_place_retreat.direction.vector.y = -1.0;
  pl.post_place_retreat.min_distance = 0.1;
  pl.post_place_retreat.desired_distance = 0.25;

  openGripper(pl.post_place_posture);

  return pl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_arm_pick_place");

  RCLCPP_INFO(node->get_logger(),
    "depth_reach stub: moveit_ros_planning_interface was not available at build time.");
  RCLCPP_INFO(node->get_logger(), "Building collision objects...");

  auto objects = buildCollisionObjects();
  RCLCPP_INFO(node->get_logger(), "Created %zu collision objects", objects.size());

  // Use transient local durability so late-joining subscribers get the messages
  auto qos = rclcpp::QoS(10).transient_local();
  auto pub = node->create_publisher<moveit_msgs::msg::CollisionObject>(
    "collision_object", qos);

  // Build grasp and place structures to exercise the message types
  auto grasp = buildGrasp();
  RCLCPP_INFO(node->get_logger(), "Grasp pose frame: %s",
    grasp.grasp_pose.header.frame_id.c_str());

  auto place_loc = buildPlaceLocation();
  RCLCPP_INFO(node->get_logger(), "Place pose frame: %s",
    place_loc.place_pose.header.frame_id.c_str());

  // Wait for a subscriber to connect (up to 15 seconds)
  auto wait_start = std::chrono::steady_clock::now();
  while (pub->get_subscription_count() == 0) {
    auto elapsed = std::chrono::steady_clock::now() - wait_start;
    if (elapsed > std::chrono::seconds(15)) {
      RCLCPP_WARN(node->get_logger(), "No subscribers after 15s, publishing anyway.");
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(node->get_logger(), "Subscriber count: %zu", pub->get_subscription_count());

  // Publish each collision object multiple times to ensure delivery
  for (int round = 0; round < 5; ++round) {
    for (auto& obj : objects) {
      obj.header.stamp = node->now();
      pub->publish(obj);
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(), "Published collision objects.");

  // Keep spinning briefly so messages are fully delivered
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(node->get_logger(), "depth_reach stub complete.");
  rclcpp::shutdown();
  return 0;
}