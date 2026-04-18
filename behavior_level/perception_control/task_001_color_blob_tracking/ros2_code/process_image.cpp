#include "rclcpp/rclcpp.hpp"
#include "ball_chaser/srv/drive_to_target.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cstdint>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Define a global client that can request services
rclcpp::Client<ball_chaser::srv::DriveToTarget>::SharedPtr client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  RCLCPP_INFO(rclcpp::get_logger("process_image"),
              "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f",
              lin_x, ang_z);

  // TODO:
  // Send a DriveToTarget service request using the global client so the robot
  // receives the desired linear and angular velocities (lin_x, ang_z).
  // If the request cannot be sent, report it with a ROS error log.
  //END OF TODO
  if (!client) {
    RCLCPP_ERROR(rclcpp::get_logger("process_image"), "Service client is not initialized.");
    return;
  }

  if (!client->wait_for_service(500ms)) {
    RCLCPP_ERROR(rclcpp::get_logger("process_image"), "Service /ball_chaser/command_robot not available.");
    return;
  }

  auto request = std::make_shared<ball_chaser::srv::DriveToTarget::Request>();
  request->linear_x = lin_x;
  request->angular_z = ang_z;

  try {
    (void)client->async_send_request(request);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("process_image"), "Failed to send service request: %s", e.what());
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
  // TODO:
  // From the incoming camera image, detect the target color blob and estimate
  // whether it appears on a side of the image.
  //
  // Then choose robot velocities:
  // Finally call drive_robot(x, z) with the chosen linear and angular speeds.
  //END OF TODO
  if (!img || img->data.empty() || img->width == 0 || img->height == 0) {
    drive_robot(0.0f, 0.0f);
    return;
  }

  const auto width = static_cast<int>(img->width);
  const auto height = static_cast<int>(img->height);
  const auto step = static_cast<int>(img->step);
  const auto & data = img->data;

  long long sum_x = 0;
  long long count = 0;

  // Detect bright (white) blob pixels
  for (int y = 0; y < height; ++y) {
    const int row_start = y * step;
    for (int x = 0; x < width; ++x) {
      const int idx = row_start + x * 3;
      if (idx + 2 >= static_cast<int>(data.size())) {
        continue;
      }

      const uint8_t c0 = data[idx + 0];
      const uint8_t c1 = data[idx + 1];
      const uint8_t c2 = data[idx + 2];

      if (c0 > 200 && c1 > 200 && c2 > 200) {
        sum_x += x;
        ++count;
      }
    }
  }

  float x = 0.0f;
  float z = 0.0f;

  if (count > 0) {
    const int centroid_x = static_cast<int>(sum_x / count);

    if (centroid_x < width / 3) {
      x = 0.0f;
      z = 0.5f;
    } else if (centroid_x > (2 * width) / 3) {
      x = 0.0f;
      z = -0.5f;
    } else {
      x = 0.5f;
      z = 0.0f;
    }
  } else {
    x = 0.0f;
    z = 0.0f;
  }

  drive_robot(x, z);
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("process_image");

  // Define a client service capable of requesting services from command_robot
  client = n->create_client<ball_chaser::srv::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  auto sub1 = n->create_subscription<sensor_msgs::msg::Image>(
    "/camera/rgb/image_raw",
    10,
    process_image_callback
  );
  RCLCPP_INFO(n->get_logger(), "Ready to receive images");

  // Handle ROS communication events
  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}