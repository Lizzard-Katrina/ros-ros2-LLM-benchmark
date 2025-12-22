#include "smacc_demo/demo_states.hpp"
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>

StateA::StateA() {}
void StateA::onEntry()
{
  ROS_INFO("StateA: onEntry - doing work for 0.1s");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
void StateA::onExit()
{
  ROS_INFO("StateA: onExit");
}

StateB::StateB() {}
void StateB::onEntry()
{
  ROS_INFO("StateB: onEntry - doing work for 0.2s");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
void StateB::onExit()
{
  ROS_INFO("StateB: onExit");
}
