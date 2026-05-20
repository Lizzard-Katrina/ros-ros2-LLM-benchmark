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

FILE_PATH: process_image.cpp
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

[FILENAME: process_image.cpp]
#include "rclcpp/rclcpp.hpp"
#include "ball_chaser/srv/drive_to_target.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <memory>

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
    RCLCPP_ERROR(g_node->get_logger(), "Service /ball_chaser/command_robot is not available");
    return;
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(g_node, result, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(g_node->get_logger(), "Failed to send DriveToTarget service request");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
  int width = img->width;
  int height = img->height;
  int step = img->step;

  int white_pixel_count = 0;
  long long white_pixel_sum_x = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * step + x * 3;

      if (idx + 2 >= static_cast<int>(img->data.size())) {
        continue;
      }

      unsigned char red = img->data[idx];
      unsigned char green = img->data[idx + 1];
      unsigned char blue = img->data[idx + 2];

      if (red >= 250 && green >= 250 && blue >= 250) {
        white_pixel_count++;
        white_pixel_sum_x += x;
      }
    }
  }

  float linear_x = 0.0;
  float angular_z = 0.0;

  if (white_pixel_count > 0) {
    int centroid_x = static_cast<int>(white_pixel_sum_x / white_pixel_count);
    int left_limit = width / 3;
    int right_limit = 2 * width / 3;

    if (centroid_x < left_limit) {
      linear_x = 0.0;
      angular_z = 0.5;
    } else if (centroid_x > right_limit) {
      linear_x = 0.0;
      angular_z = -0.5;
    } else {
      linear_x = 0.5;
      angular_z = 0.0;
    }
  }

  drive_robot(linear_x, angular_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("process_image");

    // Define a client service capable of requesting services from command_robot
    client = g_node->create_client<ball_chaser::srv::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    auto sub1 = g_node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/rgb/image_raw",
        10,
        process_image_callback);

    RCLCPP_INFO(g_node->get_logger(), "Ready to receive images");

    // Handle ROS communication events
    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}