template<typename T>
void RosFilter<T>::loadParams()
{
  // Clear any existing subscriptions
  topic_subs_.clear();

  // Sensor types to check
  const std::vector<std::string> sensor_types = {
    "odom", "pose", "twist", "imu", "accel"
  };

  // For each sensor type, check for parameters like odom0, odom1, etc.
  for (const auto & sensor_type : sensor_types)
  {
    int index = 0;
    while (true)
    {
      // Construct parameter name for topic
      std::string topic_param = sensor_type + std::to_string(index);

      // Check if parameter exists
      if (!this->has_parameter(topic_param))
      {
        // No more sensors of this type
        break;
      }

      // Get the topic name parameter
      std::string topic_name;
      this->get_parameter(topic_param, topic_name);

      // Construct config parameter name
      std::string config_param = sensor_type + std::to_string(index) + "_config";

      // Declare config parameter if not declared
      if (!this->has_parameter(config_param))
      {
        this->declare_parameter<std::vector<bool>>(config_param, std::vector<bool>(15, false));
      }

      // Get the config vector
      std::vector<bool> config_vector;
      if (!this->get_parameter(config_param, config_vector))
      {
        RCLCPP_WARN(this->get_logger(),
          "Parameter '%s' not found or invalid. Skipping sensor '%s'.",
          config_param.c_str(), topic_name.c_str());
        ++index;
        continue;
      }

      // Validate config vector size
      if (config_vector.size() != 15)
      {
        RCLCPP_WARN(this->get_logger(),
          "Parameter '%s' must have exactly 15 elements. Skipping sensor '%s'.",
          config_param.c_str(), topic_name.c_str());
        ++index;
        continue;
      }

      // Create CallbackData for this sensor
      CallbackData cb_data;
      cb_data.topic_name_ = topic_name;
      cb_data.update_vector_ = config_vector;
      cb_data.update_sum_ = std::count(config_vector.begin(), config_vector.end(), true);
      cb_data.rejection_threshold_ = 0.0;  // Default, can be overridden later
      cb_data.relative_ = false;  // Default, can be overridden later

      // Store in topic_subs_
      topic_subs_.push_back(cb_data);

      ++index;
    }
  }
}