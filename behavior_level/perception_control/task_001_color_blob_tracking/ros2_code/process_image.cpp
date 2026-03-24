#include "rclcpp/rclcpp.hpp"
#include "ball_chaser/srv/drive_to_target.hpp"
#include <sensor_msgs/msg/image.hpp>

using DriveToTarget = ball_chaser::srv::DriveToTarget;

class ProcessImageNode : public rclcpp::Node
{
public:
  ProcessImageNode()
  : Node("process_image")
  {
    client_ = this->create_client<DriveToTarget>("/ball_chaser/command_robot");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/rgb/image_raw", 10,
      std::bind(&ProcessImageNode::process_image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Ready to receive images");
  }

private:
  rclcpp::Client<DriveToTarget>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  void drive_robot(float lin_x, float ang_z)
  {
    RCLCPP_INFO(this->get_logger(), "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

    auto request = std::make_shared<DriveToTarget::Request>();
    request->linear_x = lin_x;
    request->angular_z = ang_z;

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service /ball_chaser/command_robot not available");
      return;
    }

    auto result_future = client_->async_send_request(request);
    // Optionally handle the response asynchronously or ignore
  }

  void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
  {
    int white_pixel = 255;
    int img_size = img->height * img->width * img->step;

    int left_bound = img->width / 3;
    int right_bound = 2 * img->width / 3;

    bool found = false;
    int pixel_pos = 0;

    // Iterate over each pixel (assuming RGB8 encoding)
    for (int i = 0; i < img_size; i += 3) {
      if (img->data[i] == white_pixel && img->data[i + 1] == white_pixel && img->data[i + 2] == white_pixel) {
        pixel_pos = (i / 3) % img->width;
        found = true;
        break;
      }
    }

    float lin_x = 0.0;
    float ang_z = 0.0;

    if (found) {
      if (pixel_pos < left_bound) {
        lin_x = 0.0;
        ang_z = 0.5;
      } else if (pixel_pos > right_bound) {
        lin_x = 0.0;
        ang_z = -0.5;
      } else {
        lin_x = 0.5;
        ang_z = 0.0;
      }
    } else {
      lin_x = 0.0;
      ang_z = 0.0;
    }

    drive_robot(lin_x, ang_z);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessImageNode>());
  rclcpp::shutdown();
  return 0;
}