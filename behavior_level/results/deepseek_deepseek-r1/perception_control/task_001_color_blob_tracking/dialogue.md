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
#include <sensor_msgs/msg/image.hpp>

class ProcessImageNode : public rclcpp::Node
{
public:
    ProcessImageNode() : Node("process_image")
    {
        client_ = this->create_client<ball_chaser::srv::DriveToTarget>("/ball_chaser/command_robot");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&ProcessImageNode::process_image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Ready to receive images");
    }

    void drive_robot(float lin_x, float ang_z)
    {
        RCLCPP_INFO(this->get_logger(), "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);
        
        auto request = std::make_shared<ball_chaser::srv::DriveToTarget::Request>();
        request->linear_x = lin_x;
        request->angular_z = ang_z;
        
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }
        
        auto future = client_->async_send_request(request);
        try {
            auto response = future.get();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        int white_pixel = 255;
        int ball_position = -1;
        
        for (unsigned int i = 0; i < img->height * img->step; i += 3) {
            if (img->data[i] == white_pixel && img->data[i + 1] == white_pixel && img->data[i + 2] == white_pixel) {
                ball_position = i % img->step;
                break;
            }
        }
        
        float lin_x = 0.0;
        float ang_z = 0.0;
        
        if (ball_position == -1) {
            lin_x = 0.0;
            ang_z = 0.0;
        } else if (ball_position < img->step / 3) {
            lin_x = 0.0;
            ang_z = 0.5;
        } else if (ball_position > 2 * img->step / 3) {
            lin_x = 0.0;
            ang_z = -0.5;
        } else {
            lin_x = 0.5;
            ang_z = 0.0;
        }
        
        drive_robot(lin_x, ang_z);
    }

private:
    rclcpp::Client<ball_chaser::srv::DriveToTarget>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}