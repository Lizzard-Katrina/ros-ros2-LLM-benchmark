# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: turtlebot3.cpp
----------------------------
// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string & usb_port)
: Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors * TurtleBot3::get_motors()
{
  return &motors_;
}

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  // TODO (Task 4.1): 
  // [Architectural Requirements]:
  // 1. Use 'rclcpp::AsyncParametersClient' to observe parameter events of this node.
  // 2. Ensure the system waits for the parameter service to be ready (up to 1s) 
  //    before proceeding; handle potential interruptions gracefully.
  // 3. Register a callback to process 'rcl_interfaces::msg::ParameterEvent'.
  //
  // [Logic Requirements]:
  // - Target Parameter: "motors.profile_acceleration".
  // - Action: When the target parameter is updated, synchronize the new value to 
  //   'motors_.profile_acceleration' after converting it based on the class's 
  //   internal acceleration constant.
  // - Feedback: Log the updated physical value with the unit "rev/min2".
  //
  // [Style Constraints]:
  // - Strict API: Use 'on_parameter_event' for subscription (do not use 'add_on_set_parameters_callback').
  // - Must use standard ROS 2 parameter message parsing.
  // END OF TODO

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_right.length
  );
}

void TurtleBot3::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.imu_re_calibration.addr,
      extern_control_table.imu_re_calibration.length,
      &reset,
      &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
    extern_control_table.device_status.addr,
    extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void TurtleBot3::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  uint8_t is_connected_bumper_1 = 0;
  uint8_t is_connected_bumper_2 = 0;
  uint8_t is_connected_illumination = 0;
  uint8_t is_connected_ir = 0;
  uint8_t is_connected_sonar = 0;

  this->declare_parameter<uint8_t>("sensors.bumper_1");
  this->declare_parameter<uint8_t>("sensors.bumper_2");
  this->declare_parameter<uint8_t>("sensors.illumination");
  this->declare_parameter<uint8_t>("sensors.ir");
  this->declare_parameter<uint8_t>("sensors.sonar");

  this->get_parameter_or<uint8_t>(
    "sensors.bumper_1",
    is_connected_bumper_1,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.bumper_2",
    is_connected_bumper_2,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.illumination",
    is_connected_illumination,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.ir",
    is_connected_ir,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.sonar",
    is_connected_sonar,
    0);

  sensors_.push_back(
    new sensors::BatteryState(
      node_handle_,
      "battery_state"));

  sensors_.push_back(
    new sensors::Imu(
      node_handle_,
      "imu",
      "magnetic_field",
      "imu_link"));

  sensors_.push_back(
    new sensors::SensorState(
      node_handle_,
      "sensor_state",
      is_connected_bumper_1,
      is_connected_bumper_2,
      is_connected_illumination,
      is_connected_ir,
      is_connected_sonar));

  dxl_sdk_wrapper_->read_data_set();
  sensors_.push_back(
    new sensors::JointState(
      node_handle_,
      dxl_sdk_wrapper_,
      "joint_states",
      "base_link"));
}

void TurtleBot3::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] =
    new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] =
    new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
  devices_["sound"] =
    new devices::Sound(node_handle_, dxl_sdk_wrapper_, "sound");
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      dxl_sdk_wrapper_->read_data_set();

      for (const auto & sensor : sensors_) {
        sensor->publish(now, dxl_sdk_wrapper_);
      }
    }
  );
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      dxl_sdk_wrapper_->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void TurtleBot3::parameter_event_callback()
{
// TODO (Task 4):
  // Implement a dynamic parameter monitoring system. 
  // 1. Initialize an AsyncParametersClient to listen for parameter changes on this node.
  // 2. Ensure the client waits for the parameter service to be available (max 1s wait cycles).
  // 3. Register a callback that monitors "motors.profile_acceleration":
  //    - When changed, extract its value as double and update 'motors_.profile_acceleration'.
  //    - Apply the conversion logic using 'motors_.profile_acceleration_constant'.
  //    - Log the change using RCLCPP_INFO with a message containing "rev/min2".
  // [Constraint]: Use ROS 2 standard Parameter Event subscription patterns.
  // END OF TODO
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = std::make_unique<TwistSubscriber>(
    node_handle_,
    "cmd_vel",
    qos,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>(
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      }
    ),
    std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>(
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->twist.linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->twist.angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s",
          msg->twist.linear.x,
          msg->twist.angular.z,
          sdk_msg.c_str());
      }
    )
  );
}

----------------------------


# LLM Response

[FILENAME: turtlebot3.cpp]
// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string & usb_port)
: Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors * TurtleBot3::get_motors()
{
  return &motors_;
}

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  auto start = this->now();
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service. Exiting.");
      return;
    }
    if ((this->now() - start) > rclcpp::Duration::from_seconds(1.0)) {
      RCLCPP_WARN(this->get_logger(), "Parameter service not available after 1s, proceeding without dynamic parameter monitoring.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
  }

  if (parameters_client_->service_is_ready()) {
    auto on_parameter_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        for (const auto & changed_parameter : event->changed_parameters) {
          if (changed_parameter.name == "motors.profile_acceleration") {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
            motors_.profile_acceleration = value;
            double physical_value = value * motors_.profile_acceleration_constant;
            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'motors.profile_acceleration' updated to %.2f rev/min2",
              physical_value);
          }
        }
      };

    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
  }

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_right.length
  );
}

void TurtleBot3::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.imu_re_calibration.addr,
      extern_control_table.imu_re_calibration.length,
      &reset,
      &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
    extern_control_table.device_status.addr,
    extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void TurtleBot3::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  uint8_t is_connected_bumper_1 = 0;
  uint8_t is_connected_bumper_2 = 0;
  uint8_t is_connected_illumination = 0;
  uint8_t is_connected_ir = 0;
  uint8_t is_connected_sonar = 0;

  this->declare_parameter<uint8_t>("sensors.bumper_1");
  this->declare_parameter<uint8_t>("sensors.bumper_2");
  this->declare_parameter<uint8_t>("sensors.illumination");
  this->declare_parameter<uint8_t>("sensors.ir");
  this->declare_parameter<uint8_t>("sensors.sonar");

  this->get_parameter_or<uint8_t>(
    "sensors.bumper_1",
    is_connected_bumper_1,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.bumper_2",
    is_connected_bumper_2,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.illumination",
    is_connected_illumination,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.ir",
    is_connected_ir,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.sonar",
    is_connected_sonar,
    0);

  sensors_.push_back(
    new sensors::BatteryState(
      node_handle_,
      "battery_state"));

  sensors_.push_back(
    new sensors::Imu(
      node_handle_,
      "imu",
      "magnetic_field",
      "imu_link"));

  sensors_.push_back(
    new sensors::SensorState(
      node_handle_,
      "sensor_state",
      is_connected_bumper_1,
      is_connected_bumper_2,
      is_connected_illumination,
      is_connected_ir,
      is_connected_sonar));

  dxl_sdk_wrapper_->read_data_set();
  sensors_.push_back(
    new sensors::JointState(
      node_handle_,
      dxl_sdk_wrapper_,
      "joint_states",
      "base_link"));
}

void TurtleBot3::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] =
    new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] =
    new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
  devices_["sound"] =
    new devices::Sound(node_handle_, dxl_sdk_wrapper_, "sound");
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      dxl_sdk_wrapper_->read_data_set();

      for (const auto & sensor : sensors_) {
        sensor->publish(now, dxl_sdk_wrapper_);
      }
    }
  );
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      dxl_sdk_wrapper_->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void TurtleBot3::parameter_event_callback()
{
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  auto start = this->now();
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service. Exiting.");
      return;
    }
    if ((this->now() - start) > rclcpp::Duration::from_seconds(1.0)) {
      RCLCPP_WARN(this->get_logger(), "Parameter service not available after 1s, proceeding without dynamic parameter monitoring.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
  }

  if (parameters_client_->service_is_ready()) {
    auto on_parameter_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        for (const auto & changed_parameter : event->changed_parameters) {
          if (changed_parameter.name == "motors.profile_acceleration") {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
            motors_.profile_acceleration = value;
            double physical_value = value * motors_.profile_acceleration_constant;
            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'motors.profile_acceleration' updated to %.2f rev/min2",
              physical_value);
          }
        }
      };

    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
  }
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = std::make_unique<TwistSubscriber>(
    node_handle_,
    "cmd_vel",
    qos,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>(
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      }
    ),
    std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>(
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->twist.linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->twist.angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s",
          msg->twist.linear.x,
          msg->twist.angular.z,
          sdk_msg.c_str());
      }
    )
  );
}