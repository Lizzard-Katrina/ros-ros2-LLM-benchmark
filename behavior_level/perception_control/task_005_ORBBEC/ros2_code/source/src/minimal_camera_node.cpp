/*
 * Minimal ROS2 camera node that publishes a dummy depth image and camera info.
 * Used for runtime testing of the translated package.
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class MinimalCameraNode : public rclcpp::Node {
public:
  MinimalCameraNode() : Node("ob_camera_node") {
    // Declare parameters matching the translated node
    this->declare_parameter<std::string>("camera_name", "camera");
    this->declare_parameter<bool>("depth_registration", false);
    this->declare_parameter<bool>("enable_point_cloud", true);
    this->declare_parameter<bool>("enable_colored_point_cloud", false);
    this->declare_parameter<std::string>("time_domain", "system");
    this->declare_parameter<bool>("enable_depth_scale", true);

    camera_name_ = this->get_parameter("camera_name").as_string();
    depth_registration_ = this->get_parameter("depth_registration").as_bool();

    // Publishers
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/" + camera_name_ + "/depth/image_raw", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/" + camera_name_ + "/depth/camera_info", 10);
    color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/" + camera_name_ + "/color/image_raw", 10);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/" + camera_name_ + "/depth/points", 10);

    // Timer to publish at 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalCameraNode::publishFrame, this));

    RCLCPP_INFO(this->get_logger(), "MinimalCameraNode started with camera_name=%s", camera_name_.c_str());
  }

private:
  void publishFrame() {
    auto now = this->now();

    // Publish depth image (640x480, 16UC1)
    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = now;
    depth_msg->header.frame_id = camera_name_ + "_depth_optical_frame";
    depth_msg->width = 640;
    depth_msg->height = 480;
    depth_msg->encoding = "16UC1";
    depth_msg->is_bigendian = false;
    depth_msg->step = 640 * 2;
    depth_msg->data.resize(640 * 480 * 2, 0);
    // Fill with a known pattern: pixel value = 1000 (1 meter at 1mm scale)
    uint16_t depth_val = 1000;
    for (size_t i = 0; i < 640 * 480; ++i) {
      depth_msg->data[i * 2] = depth_val & 0xFF;
      depth_msg->data[i * 2 + 1] = (depth_val >> 8) & 0xFF;
    }
    depth_image_pub_->publish(std::move(depth_msg));

    // Publish camera info
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    info_msg->header.stamp = now;
    info_msg->header.frame_id = camera_name_ + "_depth_optical_frame";
    info_msg->width = 640;
    info_msg->height = 480;
    info_msg->distortion_model = "plumb_bob";
    info_msg->d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info_msg->k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
    info_msg->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info_msg->p = {500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    depth_info_pub_->publish(std::move(info_msg));

    // Publish color image (640x480, RGB8)
    auto color_msg = std::make_unique<sensor_msgs::msg::Image>();
    color_msg->header.stamp = now;
    color_msg->header.frame_id = camera_name_ + "_color_optical_frame";
    color_msg->width = 640;
    color_msg->height = 480;
    color_msg->encoding = "rgb8";
    color_msg->is_bigendian = false;
    color_msg->step = 640 * 3;
    color_msg->data.resize(640 * 480 * 3, 128);
    color_image_pub_->publish(std::move(color_msg));
  }

  std::string camera_name_;
  bool depth_registration_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}