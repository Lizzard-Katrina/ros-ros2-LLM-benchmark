/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#pragma once

// Minimal header for ArduPilotPlugin - enough for source analysis

#include <array>
#include <memory>
#include <string>

// Minimal stubs if gz-sim headers are not available
// The actual plugin requires gz-sim but for source-level testing we
// only need the .cc file to exist with correct logic.

namespace sdf {
  class Element;
  typedef std::shared_ptr<Element> ElementPtr;
}

namespace gz {
namespace sim {

using Entity = uint64_t;
constexpr Entity kNullEntity = 0;

namespace systems {

class ArduPilotPluginPrivate;

}  // namespace systems
}  // namespace sim
}  // namespace gz

// Servo packet structures used by the plugin
#pragma pack(push, 1)
struct servo_packet_16 {
    uint16_t magic;
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

struct servo_packet_32 {
    uint16_t magic;
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[32];
};
#pragma pack(pop)