#include <lex_common_msgs/KeyValue.h>
#include <lex_node/lex_node.h>

#include <algorithm>
#include <iostream>
#include <utility>

namespace Aws {
namespace Lex {

LexRequest& operator<<(LexRequest& out_request, const lex_common_msgs::AudioTextConversationRequest& ros_request) {
  out_request.accept_type = ros_request.accept_type;
  out_request.audio_request = ros_request.audio_request.data;
  out_request.content_type = ros_request.content_type;
  out_request.text_request = ros_request.text_request;
  return out_request;
}

lex_common_msgs::AudioTextConversationResponse& operator<<(lex_common_msgs::AudioTextConversationResponse& ros_response,
                                            const LexResponse& lex_response) {
  ros_response.audio_response.data = lex_response.audio_response;
  ros_response.dialog_state = lex_response.dialog_state;
  ros_response.intent_name = lex_response.intent_name;
  ros_response.message_format_type = lex_response.message_format_type;
  ros_response.text_response = lex_response.text_response;
  ros_response.slots = std::vector<lex_common_msgs::KeyValue>();
  std::transform(lex_response.slots.begin(), lex_response.slots.end(),
                 std::back_inserter(ros_response.slots), [](const std::pair<std::string, std::string>& slot) {
              lex_common_msgs::KeyValue key_value;
              key_value.key = slot.first;
              key_value.value = slot.second;
              return key_value;
          });
  return ros_response;
}

LexNode::LexNode() : node_handle_("~") {}

ErrorCode LexNode::Init(std::shared_ptr<PostContentInterface> post_content)
{
  if (!post_content) {
    return ErrorCode::INVALID_ARGUMENT;
  }
  post_content_ = post_content;
  lex_server_ =
    node_handle_.advertiseService<>("lex_conversation", &LexNode::LexServerCallback, this);
  return ErrorCode::SUCCESS;
}

bool LexNode::LexServerCallback(lex_common_msgs::AudioTextConversationRequest & request,
                                lex_common_msgs::AudioTextConversationResponse & response)
{
// TODO(007-lex-service-loop):
// This logic must:
// 1. Translate the incoming ROS AudioTextConversationRequest into a LexRequest
// 2. Send the request to the AWS Lex service via PostContentInterface
// 3. Handle success and failure cases of the Lex request
// 4. Translate the LexResponse back into a ROS AudioTextConversationResponse
// 5. Return a boolean indicating whether the service call succeeded
// END OF TODO
}

}  // namespace Lex
}  // namespace Aws
