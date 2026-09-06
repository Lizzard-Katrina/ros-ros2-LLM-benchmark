// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/msg/pose.hpp>

#include "turtlesim/qos.hpp"

#define PI 3.141592f

class DrawSquare final : public rclcpp::Node
{
public:
  explicit DrawSquare(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_square", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawSquare::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;
  }

private:
  enum State { FORWARD, TURN, STOP };

  void poseCallback(const turtlesim::msg::Pose::ConstSharedPtr pose)
  {
    current_pose_ = *pose;
    first_pose_set_ = true;
  }

  bool hasReachedGoal()
  {
    if (state_ == FORWARD) {
      float dx = current_pose_.x - goal_pose_.x;
      float dy = current_pose_.y - goal_pose_.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      return dist < 0.1f;
    } else if (state_ == TURN) {
      // Compute angular difference
      float angle_diff = goal_pose_.theta - current_pose_.theta;
      // Wrap to [-PI, PI]
      while (angle_diff > PI) angle_diff -= 2.0f * PI;
      while (angle_diff < -PI) angle_diff += 2.0f * PI;
      return std::fabs(angle_diff) < 0.01f;
    }
    return false;
  }

  float normalizeAngle(float angle)
  {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
  }

  void stopForward()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twist_pub_->publish(twist);

    // Set up turn goal: rotate PI/2
    goal_pose_.theta = normalizeAngle(current_pose_.theta + PI / 2.0f);
    state_ = TURN;
  }

  void stopTurn()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twist_pub_->publish(twist);

    // Set up forward goal: move 2 meters in the current heading direction
    goal_pose_.x = current_pose_.x + 2.0f * std::cos(current_pose_.theta);
    goal_pose_.y = current_pose_.y + 2.0f * std::sin(current_pose_.theta);
    state_ = FORWARD;
  }

  void commandTurtle()
  {
    geometry_msgs::msg::Twist twist;
    if (state_ == FORWARD) {
      twist.linear.x = 1.0;
      twist.angular.z = 0.0;
    } else if (state_ == TURN) {
      float angle_diff = goal_pose_.theta - current_pose_.theta;
      while (angle_diff > PI) angle_diff -= 2.0f * PI;
      while (angle_diff < -PI) angle_diff += 2.0f * PI;
      twist.linear.x = 0.0;
      twist.angular.z = (angle_diff > 0) ? 0.4 : -0.4;
    }
    twist_pub_->publish(twist);
  }

  void timerCallback()
  {
    if (!first_pose_set_) {
      return;
    }

    if (!first_goal_set_) {
      first_goal_set_ = true;
      state_ = FORWARD;
      goal_pose_.x = current_pose_.x + 2.0f * std::cos(current_pose_.theta);
      goal_pose_.y = current_pose_.y + 2.0f * std::sin(current_pose_.theta);
      goal_pose_.theta = current_pose_.theta;
    }

    if (hasReachedGoal()) {
      if (state_ == FORWARD) {
        stopForward();
      } else if (state_ == TURN) {
        stopTurn();
      }
    } else {
      commandTurtle();
    }
  }

  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = FORWARD;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<DrawSquare>();

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}