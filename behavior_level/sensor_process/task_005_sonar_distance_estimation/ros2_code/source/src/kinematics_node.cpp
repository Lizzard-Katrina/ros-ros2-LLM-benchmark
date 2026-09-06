// A standalone ROS2 node that implements the same kinematics and sonar logic
// from turtle.cpp, but without Qt dependencies, for testing purposes.

#include <cmath>
#include <string>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

static const double PI = 3.14159265358979323846;
static const double TWO_PI = 2.0 * PI;

static double normalizeAngle(double angle)
{
  return angle - (TWO_PI * std::floor((angle + PI) / (TWO_PI)));
}

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("kinematics_node"),
    pos_x_(5.544445),
    pos_y_(5.544445),
    orient_(0.0),
    lin_vel_x_(0.0),
    lin_vel_y_(0.0),
    ang_vel_(0.0),
    canvas_width_(11.088889),
    canvas_height_(11.088889)
  {
    this->declare_parameter("holonomic", false);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&KinematicsNode::velocityCallback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pose_array", 10);
    sonar_pub_ = this->create_publisher<std_msgs::msg::Float64>("sonar_distance", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(16),
      std::bind(&KinematicsNode::update, this));

    RCLCPP_INFO(this->get_logger(), "Kinematics node started");
  }

private:
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
  {
    lin_vel_x_ = vel->linear.x;
    bool holonomic = false;
    this->get_parameter("holonomic", holonomic);
    if (holonomic) {
      lin_vel_y_ = vel->linear.y;
    } else {
      lin_vel_y_ = 0.0;
    }
    ang_vel_ = vel->angular.z;
  }

  void update()
  {
    double dt = 0.016;

    // Update orientation
    orient_ = normalizeAngle(orient_ + ang_vel_ * dt);

    bool holonomic = false;
    this->get_parameter("holonomic", holonomic);

    // Holonomic kinematics with rotation matrix
    if (holonomic) {
      pos_x_ += std::cos(orient_) * lin_vel_x_ * dt - std::sin(orient_) * lin_vel_y_ * dt;
      pos_y_ += -std::sin(orient_) * lin_vel_x_ * dt - std::cos(orient_) * lin_vel_y_ * dt;
    } else {
      pos_x_ += std::cos(orient_) * lin_vel_x_ * dt;
      pos_y_ += -std::sin(orient_) * lin_vel_x_ * dt;
    }

    // Boundary clamping
    if (pos_x_ < 0.0 || pos_x_ > canvas_width_ ||
        pos_y_ < 0.0 || pos_y_ > canvas_height_)
    {
      RCLCPP_WARN(this->get_logger(), "Oh no! I hit the wall!");
    }
    pos_x_ = std::min(std::max(pos_x_, 0.0), canvas_width_);
    pos_y_ = std::min(std::max(pos_y_, 0.0), canvas_height_);

    // Sonar sensing
    const double fov = 30.0 * PI / 180.0;
    const int num_rays = 11;
    const double max_range = 5.0;
    double sonar_distance = max_range;

    for (int i = 0; i < num_rays; ++i) {
      double ray_angle = orient_ - fov / 2.0 +
        fov * static_cast<double>(i) / static_cast<double>(num_rays - 1);

      double dx = std::cos(ray_angle);
      double dy = -std::sin(ray_angle);

      double min_dist = max_range;

      if (std::abs(dx) > 1e-6) {
        double t_right = (canvas_width_ - pos_x_) / dx;
        if (t_right > 0.0 && t_right < min_dist) {
          min_dist = t_right;
        }
        double t_left = (0.0 - pos_x_) / dx;
        if (t_left > 0.0 && t_left < min_dist) {
          min_dist = t_left;
        }
      }

      if (std::abs(dy) > 1e-6) {
        double t_bottom = (canvas_height_ - pos_y_) / dy;
        if (t_bottom > 0.0 && t_bottom < min_dist) {
          min_dist = t_bottom;
        }
        double t_top = (0.0 - pos_y_) / dy;
        if (t_top > 0.0 && t_top < min_dist) {
          min_dist = t_top;
        }
      }

      if (min_dist < sonar_distance) {
        sonar_distance = min_dist;
      }
    }

    sonar_distance = std::min(sonar_distance, max_range);

    // Publish pose: [x, y_flipped, theta, lin_vel, ang_vel]
    auto pose_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    pose_msg->data.resize(5);
    pose_msg->data[0] = pos_x_;
    pose_msg->data[1] = canvas_height_ - pos_y_;  // Y-flip for ROS convention
    pose_msg->data[2] = orient_;
    pose_msg->data[3] = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
    pose_msg->data[4] = ang_vel_;
    pose_pub_->publish(std::move(pose_msg));

    // Publish sonar distance
    auto sonar_msg = std::make_unique<std_msgs::msg::Float64>();
    sonar_msg->data = sonar_distance;
    sonar_pub_->publish(std::move(sonar_msg));
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sonar_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pos_x_, pos_y_, orient_;
  double lin_vel_x_, lin_vel_y_, ang_vel_;
  double canvas_width_, canvas_height_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}