#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "task_001_color_blob_tracking/srv/drive_to_target.hpp"

#include <memory>

class DriveBot : public rclcpp::Node
{
public:
  DriveBot()
  : Node("drive_bot")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    service_ = this->create_service<task_001_color_blob_tracking::srv::DriveToTarget>(
      "/ball_chaser/command_robot",
      std::bind(&DriveBot::handle_drive_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Ready to send drive commands");
  }

private:
  void handle_drive_request(
    const std::shared_ptr<task_001_color_blob_tracking::srv::DriveToTarget::Request> request,
    std::shared_ptr<task_001_color_blob_tracking::srv::DriveToTarget::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "DriveToTargetRequest received - linear.x:%1.2f, angular.z:%1.2f",
      request->linear_x, request->angular_z);

    geometry_msgs::msg::Twist motor_command;
    motor_command.linear.x = request->linear_x;
    motor_command.angular.z = request->angular_z;
    publisher_->publish(motor_command);

    response->msg_feedback = "Wheel velocity set - linear.x: "
      + std::to_string(request->linear_x)
      + " , angular.z: "
      + std::to_string(request->angular_z);

    RCLCPP_INFO(this->get_logger(), "%s", response->msg_feedback.c_str());
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<task_001_color_blob_tracking::srv::DriveToTarget>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveBot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}