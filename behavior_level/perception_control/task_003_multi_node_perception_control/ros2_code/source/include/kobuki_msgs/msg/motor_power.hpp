#ifndef KOBUKI_MSGS__MSG__MOTOR_POWER_HPP_
#define KOBUKI_MSGS__MSG__MOTOR_POWER_HPP_

#include <cstdint>

namespace kobuki_msgs
{
namespace msg
{

struct MotorPower
{
  static constexpr uint8_t ON = 1;
  static constexpr uint8_t OFF = 0;
  uint8_t state{0};
};

}  // namespace msg
}  // namespace kobuki_msgs

#endif  // KOBUKI_MSGS__MSG__MOTOR_POWER_HPP_