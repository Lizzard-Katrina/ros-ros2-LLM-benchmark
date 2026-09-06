#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Aws {
namespace Lex {

enum ErrorCode {
  SUCCESS = 0,
  INVALID_ARGUMENT = 1,
  FAILED_POST_CONTENT = 2
};

struct LexRequest {
  std::string accept_type;
  std::vector<uint8_t> audio_request;
  std::string content_type;
  std::string text_request;
};

struct LexResponse {
  std::vector<uint8_t> audio_response;
  std::string dialog_state;
  std::string intent_name;
  std::string message_format_type;
  std::string text_response;
  std::string session_attributes;
  std::vector<std::pair<std::string, std::string>> slots;
};

class PostContentInterface {
public:
  virtual ~PostContentInterface() = default;
  virtual ErrorCode PostContent(const LexRequest& request, LexResponse& response) = 0;
};

}  // namespace Lex
}  // namespace Aws