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
#include <lex_node/lex_node.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <utility>

namespace Aws {
namespace Lex {

LexNode::LexNode()
: rclcpp::Node("lex_node")
{
  this->declare_parameter<std::string>("lex_configuration_name", "default_config");
}

ErrorCode LexNode::Init(std::shared_ptr<PostContentInterface> post_content)
{
  if (!post_content) {
    return ErrorCode::INVALID_ARGUMENT;
  }
  post_content_ = post_content;
  lex_server_ = this->create_service<lex_common_msgs::srv::AudioTextConversation>(
    "lex_conversation",
    std::bind(&LexNode::LexServerCallback, this, std::placeholders::_1, std::placeholders::_2));
  return ErrorCode::SUCCESS;
}

void LexNode::LexServerCallback(
  lex_common_msgs::srv::AudioTextConversation::Request::SharedPtr request,
  lex_common_msgs::srv::AudioTextConversation::Response::SharedPtr response)
{
  LexRequest lex_request;
  lex_request.accept_type = request->accept_type;
  lex_request.audio_request = request->audio_request.data;
  lex_request.content_type = request->content_type;
  lex_request.text_request = request->text_request;

  LexResponse lex_response;
  bool is_success = !post_content_->PostContent(lex_request, lex_response);
  if (is_success) {
    response->audio_response.data = lex_response.audio_response;
    response->dialog_state = lex_response.dialog_state;
    response->intent_name = lex_response.intent_name;
    response->message_format_type = lex_response.message_format_type;
    response->text_response = lex_response.text_response;
    response->slots = std::vector<lex_common_msgs::msg::KeyValue>();
    std::transform(lex_response.slots.begin(), lex_response.slots.end(),
                   std::back_inserter(response->slots),
                   [](const std::pair<std::string, std::string>& slot) {
                     lex_common_msgs::msg::KeyValue key_value;
                     key_value.key = slot.first;
                     key_value.value = slot.second;
                     return key_value;
                   });
  }
}

}  // namespace Lex
}  // namespace Aws