#ifndef TASK_008_STUB_HARDWARE_INTERFACE_H_
#define TASK_008_STUB_HARDWARE_INTERFACE_H_

// Minimal stubs so the code compiles without ros2_control packages installed.
// When the real packages are available the build uses them instead.

#include <string>
#include <vector>
#include <cmath>

namespace angles {
  inline double from_degrees(double deg) { return deg * M_PI / 180.0; }
  inline double to_degrees(double rad) { return rad * 180.0 / M_PI; }
}

namespace hardware_interface {

enum class return_type : uint8_t { OK = 0, ERROR = 1 };

constexpr char HW_IF_POSITION[] = "position";
constexpr char HW_IF_VELOCITY[] = "velocity";

struct InterfaceInfo {
  std::string name;
};

struct ComponentInfo {
  std::string name;
  std::vector<InterfaceInfo> command_interfaces;
  std::vector<InterfaceInfo> state_interfaces;
};

struct HardwareInfo {
  std::string name;
  std::string type;
  std::vector<ComponentInfo> joints;
};

struct CallbackReturn {
  static constexpr int SUCCESS = 0;
  static constexpr int ERROR = 1;
  int value;
  CallbackReturn() : value(SUCCESS) {}
  CallbackReturn(int v) : value(v) {}
  bool operator==(const CallbackReturn& o) const { return value == o.value; }
  bool operator!=(const CallbackReturn& o) const { return value != o.value; }
};

class StateInterface {
public:
  StateInterface(const std::string& name, const std::string& iface, double* ptr)
    : name_(name), interface_(iface), ptr_(ptr) {}
  std::string name_;
  std::string interface_;
  double* ptr_;
};

class CommandInterface {
public:
  CommandInterface(const std::string& name, const std::string& iface, double* ptr)
    : name_(name), interface_(iface), ptr_(ptr) {}
  std::string name_;
  std::string interface_;
  double* ptr_;
};

class SystemInterface {
public:
  virtual ~SystemInterface() = default;

  virtual CallbackReturn on_init(const HardwareInfo & info) {
    info_ = info;
    return CallbackReturn{CallbackReturn::SUCCESS};
  }

  virtual std::vector<StateInterface> export_state_interfaces() = 0;
  virtual std::vector<CommandInterface> export_command_interfaces() = 0;

  virtual return_type read(const rclcpp::Time &, const rclcpp::Duration &) = 0;
  virtual return_type write(const rclcpp::Time &, const rclcpp::Duration &) = 0;

protected:
  HardwareInfo info_;
};

}  // namespace hardware_interface

// Stub for PLUGINLIB_EXPORT_CLASS when pluginlib is not available
#ifndef PLUGINLIB_EXPORT_CLASS
#define PLUGINLIB_EXPORT_CLASS(class_type, base_type)
#endif

#endif  // TASK_008_STUB_HARDWARE_INTERFACE_H_