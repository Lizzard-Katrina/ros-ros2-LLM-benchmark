/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>
#include <lex_node/lex_node.h>
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto lex_node = std::make_shared<Aws::Lex::LexNode>();

  RCLCPP_INFO(lex_node->get_logger(), "Starting Lex Node...");

  rclcpp::spin(lex_node);

  RCLCPP_INFO(lex_node->get_logger(), "Shutting down Lex Node...");
  rclcpp::shutdown();

  return 0;
}