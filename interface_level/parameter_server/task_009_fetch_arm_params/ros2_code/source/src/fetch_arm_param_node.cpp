/**
 * A simple ROS 2 node that demonstrates the parameter declaration pattern
 * used in the IKFast plugin initialization. This node can be launched
 * independently for runtime testing.
 */
#include <rclcpp/rclcpp.hpp>
#include <string>

class FetchArmParamNode : public rclcpp::Node
{
public:
  FetchArmParamNode() : Node("fetch_arm_param_node")
  {
    // Declare parameters following the same pattern as the IKFast plugin
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<double>("kinematics_solver_timeout", 0.05);
    this->declare_parameter<int>("kinematics_solver_attempts", 3);
    this->declare_parameter<std::string>("group_name", "arm");
    this->declare_parameter<std::string>("base_frame", "torso_lift_link");
    this->declare_parameter<std::string>("tip_frame", "wrist_roll_link");
    this->declare_parameter<double>("search_discretization", 0.05);

    // Retrieve and log parameters
    std::string robot_desc = this->get_parameter("robot_description").as_string();
    double timeout = this->get_parameter("kinematics_solver_timeout").as_double();
    int attempts = this->get_parameter("kinematics_solver_attempts").as_int();
    std::string group = this->get_parameter("group_name").as_string();
    std::string base = this->get_parameter("base_frame").as_string();
    std::string tip = this->get_parameter("tip_frame").as_string();
    double disc = this->get_parameter("search_discretization").as_double();

    RCLCPP_INFO(this->get_logger(), "FetchArmParamNode initialized");
    RCLCPP_INFO(this->get_logger(), "  group_name: %s", group.c_str());
    RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base.c_str());
    RCLCPP_INFO(this->get_logger(), "  tip_frame: %s", tip.c_str());
    RCLCPP_INFO(this->get_logger(), "  kinematics_solver_timeout: %f", timeout);
    RCLCPP_INFO(this->get_logger(), "  kinematics_solver_attempts: %d", attempts);
    RCLCPP_INFO(this->get_logger(), "  search_discretization: %f", disc);
    RCLCPP_INFO(this->get_logger(), "  robot_description length: %zu", robot_desc.size());

    // Publish parameter status on a timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        RCLCPP_DEBUG(this->get_logger(), "Parameters active");
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FetchArmParamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}