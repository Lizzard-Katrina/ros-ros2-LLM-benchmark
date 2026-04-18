/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actionlib/action/two_ints.hpp"

class AddTwoIntsServer : public rclcpp::Node
{
public:
  using TwoInts = actionlib::action::TwoInts;
  using GoalHandleTwoInts = rclcpp_action::ServerGoalHandle<TwoInts>;

  AddTwoIntsServer()
  : Node("add_two_ints_server")
  {
    action_server_ = rclcpp_action::create_server<TwoInts>(
      this,
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AddTwoIntsServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&AddTwoIntsServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<TwoInts>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TwoInts::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request: a=%ld b=%ld", goal->a, goal->b);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTwoInts> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTwoInts> goal_handle)
  {
    std::thread{std::bind(&AddTwoIntsServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTwoInts> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<TwoInts::Result>();

    if (goal_handle->is_canceling()) {
      result->sum = 0;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal canceled");
      return;
    }

    result->sum = goal->a + goal->b;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded, sum=%ld", result->sum);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}