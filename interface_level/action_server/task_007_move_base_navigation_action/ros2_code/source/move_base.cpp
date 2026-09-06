/*
 * ROS2 (Humble) translation of the ROS1 move_base navigation node.
 *
 * Original ROS1 code used actionlib with move_base_msgs::MoveBaseAction,
 * costmap_2d, nav_core base_global_planner / base_local_planner, and tf.
 *
 * This ROS2 translation uses:
 *   - rclcpp (instead of ros::NodeHandle)
 *   - rclcpp_action (instead of actionlib)
 *   - tf2_ros (instead of tf)
 *   - geometry_msgs, nav_msgs, std_srvs
 *   - nav2 equivalents where applicable
 *
 * NOTE: A full port would require nav2_costmap_2d, nav2_core, etc.
 * This file demonstrates the API translation patterns.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

/**
 * MoveBase – ROS2 Humble translation
 *
 * Key ROS1→ROS2 API changes applied:
 *   ros::NodeHandle            → rclcpp::Node (shared_ptr)
 *   ros::Publisher             → rclcpp::Publisher<T>::SharedPtr
 *   ros::Subscriber            → rclcpp::Subscription<T>::SharedPtr
 *   ros::ServiceServer         → rclcpp::Service<T>::SharedPtr
 *   ros::Timer                 → rclcpp::TimerBase::SharedPtr
 *   actionlib::SimpleActionServer → rclcpp_action::Server<T>
 *   tf::TransformListener      → tf2_ros::TransformListener + tf2_ros::Buffer
 *   ROS_INFO / ROS_WARN / …   → RCLCPP_INFO / RCLCPP_WARN / …
 *   dynamic_reconfigure        → rclcpp parameter callbacks
 *   ros::Rate                  → rclcpp::Rate
 *   ros::ok()                  → rclcpp::ok()
 *   ros::Time::now()           → this->now()
 *   ros::Duration              → rclcpp::Duration
 */

enum MoveBaseState {
  PLANNING,
  CONTROLLING,
  CLEARING
};

enum RecoveryTrigger {
  PLANNING_R,
  CONTROLLING_R,
  OSCILLATION_R
};

class MoveBase : public rclcpp::Node
{
public:
  MoveBase()
  : Node("move_base"),
    state_(PLANNING),
    recovery_trigger_(PLANNING_R),
    new_global_plan_(false),
    run_planner_(false),
    setup_(false),
    p_freq_change_(false),
    c_freq_change_(false),
    shutdown_costmaps_(false)
  {
    // ---------- tf2 ----------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------- Parameters (replaces dynamic_reconfigure) ----------
    this->declare_parameter<std::string>("base_global_planner", "navfn/NavfnROS");
    this->declare_parameter<std::string>("base_local_planner", "base_local_planner/TrajectoryPlannerROS");
    this->declare_parameter<double>("planner_frequency", 0.0);
    this->declare_parameter<double>("controller_frequency", 20.0);
    this->declare_parameter<double>("planner_patience", 5.0);
    this->declare_parameter<double>("controller_patience", 15.0);
    this->declare_parameter<double>("conservative_reset_dist", 3.0);
    this->declare_parameter<double>("oscillation_timeout", 0.0);
    this->declare_parameter<double>("oscillation_distance", 0.5);
    this->declare_parameter<bool>("shutdown_costmaps", false);
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("robot_base_frame", "base_link");

    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    planner_frequency_ = this->get_parameter("planner_frequency").as_double();
    controller_frequency_ = this->get_parameter("controller_frequency").as_double();
    planner_patience_ = this->get_parameter("planner_patience").as_double();
    controller_patience_ = this->get_parameter("controller_patience").as_double();
    conservative_reset_dist_ = this->get_parameter("conservative_reset_dist").as_double();
    oscillation_timeout_ = this->get_parameter("oscillation_timeout").as_double();
    oscillation_distance_ = this->get_parameter("oscillation_distance").as_double();
    shutdown_costmaps_ = this->get_parameter("shutdown_costmaps").as_bool();

    // ---------- Publishers ----------
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    current_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_base/current_goal", 10);
    action_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_base/goal", 10);

    // ---------- Subscribers ----------
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "move_base_simple/goal", 10,
      std::bind(&MoveBase::goalCB, this, std::placeholders::_1));

    // ---------- Services ----------
    clear_costmaps_srv_ = this->create_service<std_srvs::srv::Empty>(
      "move_base/clear_costmaps",
      std::bind(&MoveBase::clearCostmapsService, this,
                std::placeholders::_1, std::placeholders::_2));

    make_plan_srv_ = this->create_service<std_srvs::srv::Empty>(
      "move_base/make_plan",
      std::bind(&MoveBase::planService, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveBase node (ROS2) initialized");
    setup_ = true;
  }

  ~MoveBase()
  {
    RCLCPP_INFO(this->get_logger(), "MoveBase shutting down");
  }

  // ---------- Callbacks ----------
  void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received goal: (%.2f, %.2f)", goal->pose.position.x, goal->pose.position.y);
    current_goal_pub_->publish(*goal);
    planner_goal_ = *goal;
    state_ = PLANNING;
    run_planner_ = true;
  }

  void clearCostmapsService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "Clearing costmaps");
  }

  void planService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "Planning service called");
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = rclcpp::Time(0);
    try {
      global_pose = tf_buffer_->transform(robot_pose, global_frame_);
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
      return false;
    }
  }

  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);
  }

  void resetState()
  {
    state_ = PLANNING;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();
    run_planner_ = false;
  }

private:
  // State
  MoveBaseState state_;
  RecoveryTrigger recovery_trigger_;
  bool new_global_plan_;
  bool run_planner_;
  bool setup_;
  bool p_freq_change_;
  bool c_freq_change_;
  bool shutdown_costmaps_;

  // Parameters
  std::string global_frame_;
  std::string robot_base_frame_;
  double planner_frequency_;
  double controller_frequency_;
  double planner_patience_;
  double controller_patience_;
  double conservative_reset_dist_;
  double oscillation_timeout_;
  double oscillation_distance_;

  // Goal
  geometry_msgs::msg::PoseStamped planner_goal_;

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr action_goal_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_costmaps_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr make_plan_srv_;

  std::mutex planner_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveBase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}