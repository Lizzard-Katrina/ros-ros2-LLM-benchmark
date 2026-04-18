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
#include <turtlesim_msgs/msg/pose.hpp>

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
      this->create_subscription<turtlesim_msgs::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawSquare::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;
  }

private:
// TODO [Hole 2]: Implement a Navigation State Machine to draw a 2x2 meter square.
  // You must define the movement states (e.g., FORWARD, TURN) and implement the 
  // transition logic within a timer callback. 
  // Requirements:
  // 1. Use 'current_pose_' to determine if the turtle reached 'goal_pose_'.
  // 2. Linear tolerance < 0.1, Angular tolerance < 0.01.
  // 3. After each 2-meter side, rotate the turtle exactly PI/2.
  // 4. Ensure theta is wrapped correctly and velocity is published to 'turtle1/cmd_vel'.
//END OF TODO

  turtlesim_msgs::msg::Pose current_pose_;
  turtlesim_msgs::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = FORWARD;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_sub_;
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
