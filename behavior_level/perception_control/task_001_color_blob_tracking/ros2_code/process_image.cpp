#include "rclcpp/rclcpp.hpp"
#include "ball_chaser/srv/drive_to_target.hpp"
#include "sensor_msgs/msg/image.hpp"

class ProcessImage : public rclcpp::Node
{
public:
    ProcessImage() : Node("process_image")
    {
        client_ = this->create_client<ball_chaser::srv::DriveToTarget>("/ball_chaser/command_robot");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&ProcessImage::process_image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Ready to receive images");
    }

private:
    void drive_robot(float lin_x, float ang_z)
    {
        RCLCPP_INFO(this->get_logger(), "Driving robot to target - linear.x:%1.2f, angular.z:%1.2f", lin_x, ang_z);

        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service /ball_chaser/command_robot is not available.");
            return;
        }

        auto request = std::make_shared<ball_chaser::srv::DriveToTarget::Request>();
        request->linear_x = lin_x;
        request->angular_z = ang_z;

        client_->async_send_request(request);
    }

    void process_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {  
        int white_pixel = 255;
        bool ball_found = false;
        int column = 0;

        for (size_t i = 0; i < img->height * img->step; i += 3) {
            if (img->data[i] == white_pixel && img->data[i + 1] == white_pixel && img->data[i + 2] == white_pixel) {
                column = i % img->step;
                ball_found = true;
                break;
            }
        }

        if (ball_found) {
            if (column < img->step / 3) {
                drive_robot(0.0, 0.5); // Left
            } else if (column > (img->step * 2) / 3) {
                drive_robot(0.0, -0.5); // Right
            } else {
                drive_robot(0.5, 0.0); // Center
            }
        } else {
            drive_robot(0.0, 0.0); // Stop
        }
    }

    rclcpp::Client<ball_chaser::srv::DriveToTarget>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessImage>());
    rclcpp::shutdown();
    return 0;
}