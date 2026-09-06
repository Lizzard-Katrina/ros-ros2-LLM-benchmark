#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "task_001_color_blob_tracking/srv/drive_to_target.hpp"

#include <memory>
#include <functional>

class ProcessImage : public rclcpp::Node
{
public:
  ProcessImage()
  : Node("process_image")
  {
    client_ = this->create_client<task_001_color_blob_tracking::srv::DriveToTarget>(
      "/ball_chaser/command_robot");

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/rgb/image_raw", 10,
      std::bind(&ProcessImage::process_image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Ready to receive images");
  }

private:
  void drive_robot(float lin_x, float ang_z)
  {
    RCLCPP_INFO(this->get_logger(),
      "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

    auto request = std::make_shared<task_001_color_blob_tracking::srv::DriveToTarget::Request>();
    request->linear_x = lin_x;
    request->angular_z = ang_z;

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }

    client_->async_send_request(request);
  }

  void process_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int pixel = -1;
    for (int i = 0; i < (int)(msg->height * msg->step); i += 3) {

      bool is_max_r = (255 == msg->data[i]);
      bool is_max_g = (255 == msg->data[i + 1]);
      bool is_max_b = (255 == msg->data[i + 2]);

      if (is_max_r && is_max_g && is_max_b) {
        pixel = i % msg->step;
        break;
      }
    }

    float x = 0.0f;
    float z = 0.0f;

    if (pixel >= 0) {
      if (pixel < (int)(msg->step / 3)) {
        z = 0.2f;  // turn left
      } else if (pixel > (int)(2 * msg->step / 3)) {
        z = -0.2f;  // turn right
      } else {
        x = 0.2f;  // go straight
      }
    }

    drive_robot(x, z);
  }

  rclcpp::Client<task_001_color_blob_tracking::srv::DriveToTarget>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessImage>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}