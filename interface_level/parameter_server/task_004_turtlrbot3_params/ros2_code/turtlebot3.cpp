void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  // Task 4.1 implementation
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_services_interface(), this->get_node_logging_interface(), this->get_node_waitables_interface(), this->get_name());
  if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Parameter service not available after waiting");
  }

  parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::SystemDefaultsQoS(),
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
      for (const auto & changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "motors.profile_acceleration") {
          if (changed_parameter.value.type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            double new_val = changed_parameter.value.double_value;
            motors_.profile_acceleration = static_cast<float>(new_val * motors_.profile_acceleration_constant);
            RCLCPP_INFO(this->get_logger(), "Updated motors.profile_acceleration: %.3f rev/min2", motors_.profile_acceleration);
          } else if (changed_parameter.value.type == rclcpp::ParameterType::PARAMETER_INTEGER) {
            double new_val = static_cast<double>(changed_parameter.value.integer_value);
            motors_.profile_acceleration = static_cast<float>(new_val * motors_.profile_acceleration_constant);
            RCLCPP_INFO(this->get_logger(), "Updated motors.profile_acceleration: %.3f rev/min2", motors_.profile_acceleration);
          }
        }
      }
    });

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_right.length
  );
}

void TurtleBot3::parameter_event_callback()
{
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_services_interface(), this->get_node_logging_interface(), this->get_node_waitables_interface(), this->get_name());
  if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Parameter service not available after waiting");
  }

  parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::SystemDefaultsQoS(),
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
      for (const auto & changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "motors.profile_acceleration") {
          if (changed_parameter.value.type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            double new_val = changed_parameter.value.double_value;
            motors_.profile_acceleration = static_cast<float>(new_val * motors_.profile_acceleration_constant);
            RCLCPP_INFO(this->get_logger(), "Updated motors.profile_acceleration: %.3f rev/min2", motors_.profile_acceleration);
          } else if (changed_parameter.value.type == rclcpp::ParameterType::PARAMETER_INTEGER) {
            double new_val = static_cast<double>(changed_parameter.value.integer_value);
            motors_.profile_acceleration = static_cast<float>(new_val * motors_.profile_acceleration_constant);
            RCLCPP_INFO(this->get_logger(), "Updated motors.profile_acceleration: %.3f rev/min2", motors_.profile_acceleration);
          }
        }
      }
    });
}