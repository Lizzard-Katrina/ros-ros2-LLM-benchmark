#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Use our local message definitions that mirror moveit_msgs
#include <task_009_moveit_planning_scene/msg/planning_scene.hpp>
#include <task_009_moveit_planning_scene/msg/collision_object.hpp>
#include <task_009_moveit_planning_scene/msg/attached_collision_object.hpp>

// ---- Aliases so the rest of the code reads as if using moveit_msgs::msg:: ----
namespace moveit_msgs {
namespace msg {
  using PlanningScene = task_009_moveit_planning_scene::msg::PlanningScene;
  using CollisionObject = task_009_moveit_planning_scene::msg::CollisionObject;
  using AttachedCollisionObject = task_009_moveit_planning_scene::msg::AttachedCollisionObject;
}  // namespace msg
}  // namespace moveit_msgs

// Keep these includes commented to show what the real code would use:
// #include <moveit_msgs/msg/planning_scene.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit_msgs/msg/attached_collision_object.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Minimal stand-in for rviz_visual_tools so the code compiles without the
// rviz_visual_tools package (which may not be installed).  The real tutorial
// uses it only for interactive prompts; here we just log and continue.
// ---------------------------------------------------------------------------
namespace rviz_visual_tools
{
class RvizVisualTools
{
public:
  RvizVisualTools(const std::string& /*base_frame*/,
                  const std::string& /*marker_topic*/,
                  const rclcpp::Node::SharedPtr& /*node*/) {}
  void loadRemoteControl() {}
  void deleteAllMarkers() {}
  void trigger() {}
  void prompt(const std::string& msg)
  {
    RCLCPP_INFO(LOGGER, "%s  (auto-continuing)", msg.c_str());
  }
};
}  // namespace rviz_visual_tools

/**
 * Wait until the planning_scene publisher has at least one subscriber.
 */
static void wait_for_subscribers(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    const rclcpp::Node::SharedPtr& node)
{
  while (rclcpp::ok() && pub->get_subscription_count() < 1)
  {
    RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 2000,
                         "Waiting for a subscriber to the planning scene topic...");
    rclcpp::sleep_for(500ms);
  }
  // Give the subscriber a moment to fully establish
  rclcpp::sleep_for(1s);
}

/**
 * Create the attached collision object (a small box).
 */
static moveit_msgs::msg::AttachedCollisionObject make_attached_box()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";

  attached_object.object.header.frame_id = "panda_hand";
  attached_object.object.id = "box";

  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  attached_object.touch_links =
      std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  return attached_object;
}

/**
 * Publish a planning scene diff message.
 */
static void publish_scene_diff(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    moveit_msgs::msg::PlanningScene& scene)
{
  scene.is_diff = true;
  pub->publish(scene);
  // Delay to ensure subscriber receives the message before next publish
  rclcpp::sleep_for(500ms);
}

/**
 * Step 1: Add the object into the world.
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
 */
static void step_attach_object_and_remove_from_world__TODO(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& pub,
    rviz_visual_tools::RvizVisualTools& visual_tools,
    const moveit_msgs::msg::AttachedCollisionObject& attached_object)
{
  RCLCPP_INFO(LOGGER, "Attaching the object to the robot and removing it from the world.");

  moveit_msgs::msg::PlanningScene planning_scene;

  // Create a CollisionObject with REMOVE operation to take the object out of the world
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  // Clear and push the REMOVE into world collision objects
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);

  // Attach the object to the robot state
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;

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

  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;

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

  rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  auto planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

  wait_for_subscribers(planning_scene_diff_publisher, node);
  visual_tools.prompt("Press 'next' in RViz to start: add object");

  const auto attached_object = make_attached_box();

  step_add_object_to_world(planning_scene_diff_publisher, visual_tools, attached_object);
  step_attach_object_and_remove_from_world__TODO(planning_scene_diff_publisher, visual_tools, attached_object);
  step_detach_object_and_return_to_world(planning_scene_diff_publisher, visual_tools, attached_object);
  step_remove_object_from_world(planning_scene_diff_publisher, visual_tools);

  rclcpp::shutdown();
  return 0;
}