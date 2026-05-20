# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: apply_planning.cpp
----------------------------
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Visualization helper used by the official tutorial
#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");
using namespace std::chrono_literals;

/**
 * Wait until the planning_scene publisher has at least one subscriber.
 * (In typical setups, move_group subscribes to "planning_scene".)
 */
static void wait_for_subscribers(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    const rclcpp::Node::SharedPtr& node)
{
  // Keep it simple: block until at least one subscriber is connected.
  while (rclcpp::ok() && pub->get_subscription_count() < 1)
  {
    RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 2000,
                         "Waiting for a subscriber to the planning scene topic...");
    rclcpp::sleep_for(500ms);
  }
}

/**
 * Create the attached collision object (a small box) to be used in the demo.
 * This matches the tutorial’s intent: a box at the hand, attachable to panda_hand.
 */
static moveit_msgs::msg::AttachedCollisionObject make_attached_box()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";

  // Header must contain a valid TF frame
  attached_object.object.header.frame_id = "panda_hand";
  attached_object.object.id = "box";

  // Default pose
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  // Box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // IMPORTANT: attaching requires ADD
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Touch links: allow collisions between the object and the gripper links
  attached_object.touch_links =
      std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  return attached_object;
}

/**
 * Publish a planning scene diff message (asynchronous update via topic).
 */
static void publish_scene_diff(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    moveit_msgs::msg::PlanningScene& scene)
{
  scene.is_diff = true;
  pub->publish(scene);
}

/**
 * Step 1: Add the object into the world (planning_scene.world.collision_objects).
 */
static void step_add_object_to_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand.");

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Press 'next' in RViz to continue: attach object");
}

/**
 * Step 2: Attach object to robot AND remove it from world.
 *
 * ROS1/ROS2 tutorial semantics:
 * - Remove original object from world (CollisionObject REMOVE)
 * - Attach object to robot_state.attached_collision_objects (ADD)
 * - Ensure diff fields are clean: clear world collision objects before pushing
 *   and set robot_state.is_diff = true.
 *
 */
static void step_attach_object_and_remove_from_world__TODO(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  (void)pub;
  (void)visual_tools;
  (void)attached_object;
//TODO:
// Update the planning scene to reflect that the object changes
// from being part of the environment to being carried by the robot.
//
// Construct and publish an appropriate planning scene diff
//
//END OF TODO
}

/**
 * Step 3: Detach object from robot and return it to world.
 */
static void step_detach_object_and_return_to_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");

  // DETACH message: remove attached object from robot state
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;

  // Make sure we do not carry other attached objects/collision objects unintentionally
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;

  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Press 'next' in RViz to continue: remove object");
}

/**
 * Step 4: Remove the object from the collision world.
 */
static void step_remove_object_from_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools)
{
  RCLCPP_INFO(LOGGER, "Removing the object from the world.");

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Demo complete. Press 'next' to exit.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial");

  // Visualization tools (matches tutorial style)
  rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Planning scene diff publisher
  auto planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  wait_for_subscribers(planning_scene_diff_publisher, node);
  visual_tools.prompt("Press 'next' in RViz to start: add object");

  // Create the attached object message (box)
  const auto attached_object = make_attached_box();

  // Step through the demo
  step_add_object_to_world(planning_scene_diff_publisher, visual_tools, attached_object);

  // HOLLOWED semantic core:
  step_attach_object_and_remove_from_world__TODO(planning_scene_diff_publisher, visual_tools, attached_object);

  step_detach_object_and_return_to_world(planning_scene_diff_publisher, visual_tools, attached_object);
  step_remove_object_from_world(planning_scene_diff_publisher, visual_tools);

  rclcpp::shutdown();
  return 0;
}

----------------------------


# LLM Response

[FILENAME: apply_planning.cpp]
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Visualization helper used by the official tutorial
#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");
using namespace std::chrono_literals;

/**
 * Wait until the planning_scene publisher has at least one subscriber.
 * (In typical setups, move_group subscribes to "planning_scene".)
 */
static void wait_for_subscribers(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    const rclcpp::Node::SharedPtr& node)
{
  // Keep it simple: block until at least one subscriber is connected.
  while (rclcpp::ok() && pub->get_subscription_count() < 1)
  {
    RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 2000,
                         "Waiting for a subscriber to the planning scene topic...");
    rclcpp::sleep_for(500ms);
  }
}

/**
 * Create the attached collision object (a small box) to be used in the demo.
 * This matches the tutorial’s intent: a box at the hand, attachable to panda_hand.
 */
static moveit_msgs::msg::AttachedCollisionObject make_attached_box()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";

  // Header must contain a valid TF frame
  attached_object.object.header.frame_id = "panda_hand";
  attached_object.object.id = "box";

  // Default pose
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  // Box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // IMPORTANT: attaching requires ADD
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Touch links: allow collisions between the object and the gripper links
  attached_object.touch_links =
      std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  return attached_object;
}

/**
 * Publish a planning scene diff message (asynchronous update via topic).
 */
static void publish_scene_diff(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    moveit_msgs::msg::PlanningScene& scene)
{
  scene.is_diff = true;
  pub->publish(scene);
}

/**
 * Step 1: Add the object into the world (planning_scene.world.collision_objects).
 */
static void step_add_object_to_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand.");

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Press 'next' in RViz to continue: attach object");
}

/**
 * Step 2: Attach object to robot AND remove it from world.
 *
 * ROS1/ROS2 tutorial semantics:
 * - Remove original object from world (CollisionObject REMOVE)
 * - Attach object to robot_state.attached_collision_objects (ADD)
 * - Ensure diff fields are clean: clear world collision objects before pushing
 *   and set robot_state.is_diff = true.
 *
 */
static void step_attach_object_and_remove_from_world__TODO(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Attaching the object to the robot and removing it from the world.");

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;

  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;

  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Press 'next' in RViz to continue: detach object");
}

/**
 * Step 3: Detach object from robot and return it to world.
 */
static void step_detach_object_and_return_to_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");

  // DETACH message: remove attached object from robot state
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;

  // Make sure we do not carry other attached objects/collision objects unintentionally
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;

  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Press 'next' in RViz to continue: remove object");
}

/**
 * Step 4: Remove the object from the collision world.
 */
static void step_remove_object_from_world(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools)
{
  RCLCPP_INFO(LOGGER, "Removing the object from the world.");

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);

  publish_scene_diff(pub, planning_scene);
  visual_tools.prompt("Demo complete. Press 'next' to exit.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial");

  // Visualization tools (matches tutorial style)
  rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Planning scene diff publisher
  auto planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  wait_for_subscribers(planning_scene_diff_publisher, node);
  visual_tools.prompt("Press 'next' in RViz to start: add object");

  // Create the attached object message (box)
  const auto attached_object = make_attached_box();

  // Step through the demo
  step_add_object_to_world(planning_scene_diff_publisher, visual_tools, attached_object);

  // HOLLOWED semantic core:
  step_attach_object_and_remove_from_world__TODO(planning_scene_diff_publisher, visual_tools, attached_object);

  step_detach_object_and_return_to_world(planning_scene_diff_publisher, visual_tools, attached_object);
  step_remove_object_from_world(planning_scene_diff_publisher, visual_tools);

  rclcpp::shutdown();
  return 0;
}