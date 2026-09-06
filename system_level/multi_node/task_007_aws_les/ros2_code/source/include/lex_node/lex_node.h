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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <task_007_aws_les/srv/audio_text_conversation.hpp>
#include <task_007_aws_les/msg/key_value.hpp>

#include <lex_node/lex_common.h>

#include <memory>
#include <string>

namespace lex_common_msgs {
namespace srv {
  using AudioTextConversation = task_007_aws_les::srv::AudioTextConversation;
}  // namespace srv
namespace msg {
  using KeyValue = task_007_aws_les::msg::KeyValue;
}  // namespace msg
}  // namespace lex_common_msgs

namespace Aws {
namespace Lex {

/**
 * LexNode is responsible for providing ROS API's and configuration for Amazon Lex.
 * The lex node will work on each incoming message serially and respond with the lex info.
 */
class LexNode : public rclcpp::Node
{
private:
  /**
   * The ros service server for lex requests.
   */
  rclcpp::Service<lex_common_msgs::srv::AudioTextConversation>::SharedPtr lex_server_;

  /**
   * Post content function.
   */
  std::shared_ptr<PostContentInterface> post_content_;

  /**
   * Service callback for lex. Only allow one interaction with Lex at a time. If a new request comes
   * in, fail the last request, then make a new request.
   *
   * @param request to handle
   * @param response to fill
   */
  void LexServerCallback(
    lex_common_msgs::srv::AudioTextConversation::Request::SharedPtr request,
    lex_common_msgs::srv::AudioTextConversation::Response::SharedPtr response);

public:
  /**
   * Constructor.
   */
  LexNode();

  /**
   * Destructor.
   */
  ~LexNode() = default;

  /**
   * Initialize the lex node.
   *
   * @param lex_interactor to use as the method to call lex.
   */
  ErrorCode Init(std::shared_ptr<PostContentInterface> lex_interactor);
};

}  // namespace Lex
}  // namespace Aws