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
#include <lex_common_msgs/msg/audio_text_conversation_request.hpp>
#include <lex_common_msgs/msg/audio_text_conversation_response.hpp>
#include <lex_node/lex_node.hpp>

#include <algorithm>
#include <iostream>
#include <utility>

namespace Aws {
namespace Lex {

LexRequest& operator<<(LexRequest& out_request, const lex_common_msgs::msg::AudioTextConversationRequest& ros_request) {
  out_request.accept_type = ros_request.accept_type;
  out_request.audio_request = ros_request.audio_request;
  out_request.content_type = ros_request.content_type;
  out_request.text_request = ros_request.text_request;
  return out_request;
}

lex_common_msgs::msg::AudioTextConversationResponse& operator<<(lex_common_msgs::msg::AudioTextConversationResponse& ros_response,
                                            const LexResponse& lex_response) {
  ros_response.audio_response = lex_response.audio_response;
  ros_response.dialog_state = lex_response.dialog_state;
  ros_response.intent_name = lex_response.intent_name;
  ros_response.message_format_type = lex_response.message_format_type;
  ros_response.text_response = lex_response.text_response;
  ros_response.slots = std::vector<lex_common_msgs::msg::KeyValue>();
  std::transform(lex_response.slots.begin(), lex_response.slots.end(),
                 std::back_inserter(ros_response.slots), [](const std::pair<std::string, std::string>& slot) {
              lex_common_msgs::msg::KeyValue key_value;
              key_value.key = slot.first;
              key_value.value = slot.second;
              return key_value;
          });
  return ros_response;
}

/**
 * Implement the complete LexNode Constructor for ROS 2.
 * - Initialize the node (if inheriting from rclcpp::Node, call the base constructor).
 * - Perform all necessary ROS 2 parameter declarations for configuration.
 * - Style Constraint: Use 'this->declare_parameter<std::string>("lex_configuration_name", "default_val")' pattern.
 */
LexNode::LexNode(const std::string & node_name)
: Node(node_name)
{
  this->declare_parameter<std::string>("lex_configuration_name", "default_val");
  this->declare_parameter<std::string>("lex_server_name", "lex_server");
}

/**
 * Implement the complete Init function logic for ROS 2.
 * - Ensure 'post_content' is valid before assignment.
 * - Initialize the 'lex_server_' member using the ROS 2 service creation pattern.
 * - Style Constraint: You MUST use 'this->create_service<...>' and bind it to 'LexServerCallback'.
 * - IMPORTANT: The logic must strictly use the member variable names defined in your updated lex_node.h.
 */
ErrorCode LexNode::Init(std::shared_ptr<PostContentInterface> post_content)
{
  if (!post_content) {
    return ErrorCode::INVALID_POST_CONTENT;
  }
  post_content_ = post_content;
  lex_server_ = this->create_service<lex_common_msgs::srv::AudioTextConversation>(
    this->get_parameter("lex_server_name").as_string(),
    std::bind(&LexNode::LexServerCallback, this, std::placeholders::_1, std::placeholders::_2));
  return ErrorCode::SUCCESS;
}

bool LexNode::LexServerCallback(const std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Request> request,
                                std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Response> response)
{
  LexRequest lex_request;
  lex_request << *request;
  LexResponse lex_response;
  bool is_success = !post_content_->PostContent(lex_request, lex_response);
  if (is_success) {
    *response << lex_response;
  }
  return is_success;
}

}  // namespace Lex
}  // namespace Aws