//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.
// BSD License
//=================================================================================================

#include <rclcpp/rclcpp.hpp>
#include "task_007_hector_slam/HectorMappingRos.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HectorMappingRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}