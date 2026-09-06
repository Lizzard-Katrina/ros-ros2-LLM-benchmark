/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar, Enrique Fernández
 * Migrated to ROS 2 (rclcpp)
 */

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace diff_drive_controller
{

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller"),
    open_loop_(false),
    wheel_separation_(0.0),
    wheel_radius_(0.0),
    wheel_separation_multiplier_(1.0),
    left_wheel_radius_multiplier_(1.0),
    right_wheel_radius_multiplier_(1.0),
    cmd_vel_timeout_(0.5),
    allow_multiple_cmd_vel_publishers_(true),
    base_frame_id_("base_link"),
    odom_frame_id_("odom"),
    enable_odom_tf_(true),
    wheel_joints_size_(0),
    publish_cmd_(false),
    publish_wheel_joint_controller_state_(false)
  {
  }

  void init(std::shared_ptr<rclcpp::Node> node)
  {
    // Odometry related:
    node->declare_parameter<double>("publish_rate", 50.0);
    double publish_rate = node->get_parameter("publish_rate").as_double();
    RCLCPP_INFO_STREAM(node->get_logger(), "Controller state will be published at " << publish_rate << "Hz.");

    // --- TODO Hole 1 & 2: Parameter declarations ---
    // Declare and retrieve 'open_loop' (bool, default: false)
    node->declare_parameter<bool>("open_loop", false);
    open_loop_ = node->get_parameter("open_loop").as_bool();

    // Declare and retrieve 'velocity_rolling_window_size' (int, default: 10)
    node->declare_parameter<int>("velocity_rolling_window_size", 10);
    int velocity_rolling_window_size = node->get_parameter("velocity_rolling_window_size").as_int();
    odometry_setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Declare and retrieve 'cmd_vel_timeout' (double, default: 0.5)
    node->declare_parameter<double>("cmd_vel_timeout", 0.5);
    cmd_vel_timeout_ = node->get_parameter("cmd_vel_timeout").as_double();

    // Log the cmd_vel_timeout value using RCLCPP_INFO_STREAM
    RCLCPP_INFO_STREAM(node->get_logger(), "Velocity commands will be considered old if they are older than "
                       << cmd_vel_timeout_ << "s.");
    // --- END OF TODO Hole 1 & 2 ---

    node->declare_parameter<std::string>("base_frame_id", base_frame_id_);
    base_frame_id_ = node->get_parameter("base_frame_id").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Base frame_id set to " << base_frame_id_);

    node->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
    odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Odometry frame_id set to " << odom_frame_id_);

    node->declare_parameter<bool>("enable_odom_tf", enable_odom_tf_);
    enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();
    RCLCPP_INFO_STREAM(node->get_logger(), "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

    // Setup odometry publisher fields
    setOdomPubFields(node);

    // Create publisher for odom
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

    // Create subscriber for cmd_vel
    sub_command_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        (void)msg;
      });

    RCLCPP_INFO_STREAM(node->get_logger(), "DiffDriveController initialized successfully.");
  }

  void setOdomPubFields(std::shared_ptr<rclcpp::Node> node)
  {
    // --- TODO Hole 3: Covariance diagonal parameter handling ---
    // Declare and retrieve pose_covariance_diagonal
    std::vector<double> default_pose_cov = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03};
    node->declare_parameter<std::vector<double>>("pose_covariance_diagonal", default_pose_cov);
    std::vector<double> pose_cov_list = node->get_parameter("pose_covariance_diagonal").as_double_array();

    if (pose_cov_list.size() != 6)
    {
      throw std::invalid_argument("diagonal size must be 6");
    }

    // Declare and retrieve twist_covariance_diagonal
    std::vector<double> default_twist_cov = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03};
    node->declare_parameter<std::vector<double>>("twist_covariance_diagonal", default_twist_cov);
    std::vector<double> twist_cov_list = node->get_parameter("twist_covariance_diagonal").as_double_array();

    if (twist_cov_list.size() != 6)
    {
      throw std::invalid_argument("diagonal size must be 6");
    }
    // --- END OF TODO Hole 3 ---

    // Setup odom message constant fields
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};

    odom_msg_ = odom_msg;

    RCLCPP_INFO_STREAM(node->get_logger(), "Odometry pub fields configured.");
  }

  // Accessors for testing
  double get_cmd_vel_timeout() const { return cmd_vel_timeout_; }
  bool get_open_loop() const { return open_loop_; }
  std::string get_base_frame_id() const { return base_frame_id_; }
  std::string get_odom_frame_id() const { return odom_frame_id_; }
  nav_msgs::msg::Odometry get_odom_msg() const { return odom_msg_; }

private:
  void odometry_setVelocityRollingWindowSize(int size)
  {
    velocity_rolling_window_size_ = size;
  }

  bool open_loop_;
  double wheel_separation_;
  double wheel_radius_;
  double wheel_separation_multiplier_;
  double left_wheel_radius_multiplier_;
  double right_wheel_radius_multiplier_;
  double cmd_vel_timeout_;
  bool allow_multiple_cmd_vel_publishers_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool enable_odom_tf_;
  size_t wheel_joints_size_;
  bool publish_cmd_;
  bool publish_wheel_joint_controller_state_;
  int velocity_rolling_window_size_{10};

  nav_msgs::msg::Odometry odom_msg_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
};

}  // namespace diff_drive_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<diff_drive_controller::DiffDriveController>();
  try
  {
    node->init(node);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Initialization failed: " << e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}