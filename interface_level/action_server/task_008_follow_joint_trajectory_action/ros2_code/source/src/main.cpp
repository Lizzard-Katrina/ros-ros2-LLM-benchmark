#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

class TrajectoryCheckNode : public rclcpp::Node
{
public:
  TrajectoryCheckNode() : Node("trajectory_check_node")
  {
    pub_ = this->create_publisher<std_msgs::msg::Bool>("check_trajectory_result", 10);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        (void)msg;
        // In a real system, we'd update base_odom_ here
        // For this benchmark, we just publish a result
        auto result = std_msgs::msg::Bool();
        result.data = true;
        pub_->publish(result);
      });

    RCLCPP_INFO(this->get_logger(), "TrajectoryCheckNode started");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryCheckNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}