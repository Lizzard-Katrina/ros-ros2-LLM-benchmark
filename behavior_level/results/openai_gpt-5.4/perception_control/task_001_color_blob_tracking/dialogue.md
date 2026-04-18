# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO("Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

    // TODO:
  // Send a DriveToTarget service request using the global client so the robot
  // receives the desired linear and angular velocities (lin_x, ang_z).
  // If the request cannot be sent, report it with a ROS error log.
  //END OF TODO

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{  

  // TODO:
  // From the incoming camera image, detect the target color blob and estimate
  // whether it appears on a side of the image.
  //
  // Then choose robot velocities:
  // Finally call drive_robot(x, z) with the chosen linear and angular speeds.
  //END OF TODO

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO("Ready to receive images");

    // Handle ROS communication events
    ros::spin();

    return 0;
}

----------------------------


# LLM Response

#include "rclcpp/rclcpp.hpp"
#include "ball_chaser/srv/drive_to_target.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <vector>
#include <string>

using std::placeholders::_1;

// Define a global client that can request services
rclcpp::Client<ball_chaser::srv::DriveToTarget>::SharedPtr client;
rclcpp::Node::SharedPtr g_node;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  RCLCPP_INFO(g_node->get_logger(), "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

  auto request = std::make_shared<ball_chaser::srv::DriveToTarget::Request>();
  request->linear_x = lin_x;
  request->angular_z = ang_z;

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(g_node->get_logger(), "command_robot service not available");
    return;
  }

  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    RCLCPP_ERROR(g_node->get_logger(), "Failed to send request to command_robot service");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
  int white_pixel = 255;
  long long left_sum = 0;
  long long right_sum = 0;

  int width = static_cast<int>(img->width);
  int height = static_cast<int>(img->height);
  int step = static_cast<int>(img->step);

  int row = height / 2;

  for (int col = 0; col < width; col++) {
    int idx = row * step + col * 3;
    if (idx + 2 >= static_cast<int>(img->data.size())) {
      continue;
    }

    int r = img->data[idx];
    int g = img->data[idx + 1];
    int b = img->data[idx + 2];

    if (r == white_pixel && g == white_pixel && b == white_pixel) {
      if (col < width / 2) {
        left_sum += r + g + b;
      } else {
        right_sum += r + g + b;
      }
    }
  }

  float x = 0.0;
  float z = 0.0;

  if (left_sum > 0 && right_sum > 0) {
    x = 0.5;
    z = 0.0;
  } else if (left_sum > 0) {
    x = 0.0;
    z = 0.5;
  } else if (right_sum > 0) {
    x = 0.0;
    z = -0.5;
  } else {
    x = 0.0;
    z = 0.0;
  }

  drive_robot(x, z);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("process_image");

  client = g_node->create_client<ball_chaser::srv::DriveToTarget>("/ball_chaser/command_robot");

  auto sub1 = g_node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/rgb/image_raw",
    10,
    process_image_callback);

  RCLCPP_INFO(g_node->get_logger(), "Ready to receive images");

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  return 0;
}