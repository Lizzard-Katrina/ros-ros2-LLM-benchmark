template<typename T>
void RosFilter<T>::loadParams()
{
  auto declare_if_needed = [this](const std::string & name, const rclcpp::ParameterValue & default_value) {
      if (!this->has_parameter(name)) {
        this->declare_parameter(name, default_value);
      }
    };

  auto count_updates = [](const std::vector<bool> & v) -> size_t {
      return static_cast<size_t>(std::count(v.begin(), v.end(), true));
    };

  auto make_update_vector = [](size_t size, const std::vector<bool> & src, size_t start, size_t end) {
      std::vector<bool> out(size, false);
      for (size_t i = start; i <= end && i < src.size() && i < out.size(); ++i) {
        out[i] = src[i];
      }
      return out;
    };

  // Core parameters
  this->get_parameter("map_frame", map_frame_id_);
  this->get_parameter("odom_frame", odom_frame_id_);
  this->get_parameter("base_link_frame", base_link_frame_id_);
  this->get_parameter("base_link_frame_output", base_link_output_frame_id_);
  this->get_parameter("world_frame", world_frame_id_);
  this->get_parameter("print_diagnostics", print_diagnostics_);
  this->get_parameter("publish_tf", publish_transform_);
  this->get_parameter("publish_acceleration", publish_acceleration_);
  this->get_parameter("permit_corrected_publication", permit_corrected_publication_);
  this->get_parameter("predict_to_current_time", predict_to_current_time_);
  this->get_parameter("two_d_mode", two_d_mode_);
  this->get_parameter("smooth_lagged_data", smooth_lagged_data_);
  this->get_parameter("reset_on_time_jump", reset_on_time_jump_);
  this->get_parameter("use_control", use_control_);
  this->get_parameter("stamped_control", stamped_control_);
  this->get_parameter("disabled_at_startup", disabled_at_startup_);
  this->get_parameter("gravitational_acceleration", gravitational_acceleration_);
  this->get_parameter("frequency", frequency_);

  double sensor_timeout_tmp = sensor_timeout_.seconds();
  this->get_parameter("sensor_timeout", sensor_timeout_tmp);
  sensor_timeout_ = rclcpp::Duration::from_seconds(sensor_timeout_tmp);

  double history_length_tmp = history_length_.seconds();
  this->get_parameter("history_length", history_length_tmp);
  history_length_ = rclcpp::Duration::from_seconds(history_length_tmp);

  double tf_timeout_tmp = tf_timeout_.seconds();
  this->get_parameter("transform_timeout", tf_timeout_tmp);
  tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_tmp);

  double tf_offset_tmp = tf_time_offset_.seconds();
  this->get_parameter("transform_time_offset", tf_offset_tmp);
  tf_time_offset_ = rclcpp::Duration::from_seconds(tf_offset_tmp);

  // Process/estimate covariance
  std::vector<double> process_noise_vec;
  if (this->get_parameter("process_noise_covariance", process_noise_vec)) {
    if (process_noise_vec.size() == STATE_SIZE * STATE_SIZE) {
      for (size_t r = 0; r < STATE_SIZE; ++r) {
        for (size_t c = 0; c < STATE_SIZE; ++c) {
          process_noise_covariance_(r, c) = process_noise_vec[r * STATE_SIZE + c];
        }
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "process_noise_covariance must have %zu elements, but has %zu. Using defaults.",
        static_cast<size_t>(STATE_SIZE * STATE_SIZE), process_noise_vec.size());
    }
  }

  std::vector<double> initial_estimate_vec;
  if (this->get_parameter("initial_estimate_covariance", initial_estimate_vec)) {
    if (initial_estimate_vec.size() == STATE_SIZE * STATE_SIZE) {
      for (size_t r = 0; r < STATE_SIZE; ++r) {
        for (size_t c = 0; c < STATE_SIZE; ++c) {
          initial_estimate_error_covariance_(r, c) = initial_estimate_vec[r * STATE_SIZE + c];
        }
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "initial_estimate_covariance must have %zu elements, but has %zu. Using defaults.",
        static_cast<size_t>(STATE_SIZE * STATE_SIZE), initial_estimate_vec.size());
    }
  }

  filter_.setSensorTimeout(sensor_timeout_);
  filter_.setProcessNoiseCovariance(process_noise_covariance_);
  filter_.setEstimateErrorCovariance(initial_estimate_error_covariance_);

  // Reset subscriptions before rebuilding from params
  topic_subs_.clear();

  // Control input subscription
  if (use_control_) {
    declare_if_needed("control_topic", rclcpp::ParameterValue(std::string("cmd_vel")));
    declare_if_needed("control_queue_size", rclcpp::ParameterValue(10));
    std::string control_topic = "cmd_vel";
    int control_queue_size = 10;
    this->get_parameter("control_topic", control_topic);
    this->get_parameter("control_queue_size", control_queue_size);

    if (stamped_control_) {
      stamped_control_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        control_topic,
        rclcpp::QoS(static_cast<size_t>(std::max(1, control_queue_size))),
        std::bind(&RosFilter<T>::controlStampedCallback, this, std::placeholders::_1));
    } else {
      control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        control_topic,
        rclcpp::QoS(static_cast<size_t>(std::max(1, control_queue_size))),
        std::bind(&RosFilter<T>::controlCallback, this, std::placeholders::_1));
    }
  }

  constexpr size_t kMaxIndexedSensors = 100;

  // Odom sensors
  for (size_t i = 0; i < kMaxIndexedSensors; ++i) {
    const std::string sensor_name = "odom" + std::to_string(i);
    const std::string config_name = sensor_name + "_config";
    const std::string queue_name = sensor_name + "_queue_size";
    const std::string relative_name = sensor_name + "_relative";
    const std::string differential_name = sensor_name + "_differential";
    const std::string rejection_name = sensor_name + "_rejection_threshold";

    declare_if_needed(sensor_name, rclcpp::ParameterValue(std::string("")));
    std::string topic;
    this->get_parameter(sensor_name, topic);
    if (topic.empty()) {
      break;
    }

    declare_if_needed(config_name, rclcpp::ParameterValue(std::vector<bool>(STATE_SIZE, false)));
    std::vector<bool> config;
    this->get_parameter(config_name, config);

    if (config.size() != STATE_SIZE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid %s size: expected 15, got %zu. Sensor will be skipped.",
        config_name.c_str(), config.size());
      continue;
    }

    declare_if_needed(queue_name, rclcpp::ParameterValue(10));
    declare_if_needed(relative_name, rclcpp::ParameterValue(false));
    declare_if_needed(differential_name, rclcpp::ParameterValue(false));
    declare_if_needed(rejection_name, rclcpp::ParameterValue(std::numeric_limits<double>::max()));

    int queue_size = 10;
    bool relative = false;
    bool differential = false;
    double rejection_threshold = std::numeric_limits<double>::max();
    this->get_parameter(queue_name, queue_size);
    this->get_parameter(relative_name, relative);
    this->get_parameter(differential_name, differential);
    this->get_parameter(rejection_name, rejection_threshold);

    CallbackData pose_cb;
    pose_cb.topic_name_ = topic;
    pose_cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberX, StateMemberYaw);
    pose_cb.update_sum_ = count_updates(pose_cb.update_vector_);
    pose_cb.relative_ = relative;
    pose_cb.differential_ = differential;
    pose_cb.rejection_threshold_ = rejection_threshold;

    CallbackData twist_cb;
    twist_cb.topic_name_ = topic;
    twist_cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberVx, StateMemberVyaw);
    twist_cb.update_sum_ = count_updates(twist_cb.update_vector_);
    twist_cb.relative_ = relative;
    twist_cb.differential_ = differential;
    twist_cb.rejection_threshold_ = rejection_threshold;

    auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
      topic,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, queue_size))),
      [this, pose_cb, twist_cb](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (pose_cb.update_sum_ > 0) {
          auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
          pose_msg->header = msg->header;
          pose_msg->pose = msg->pose;
          this->poseCallback(
            pose_msg, pose_cb, this->world_frame_id_, this->base_link_frame_id_, false);
        }

        if (twist_cb.update_sum_ > 0) {
          auto twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
          twist_msg->header = msg->header;
          twist_msg->twist = msg->twist;
          this->twistCallback(twist_msg, twist_cb, this->base_link_frame_id_);
        }
      });

    topic_subs_.push_back(sub);
  }

  // Pose sensors
  for (size_t i = 0; i < kMaxIndexedSensors; ++i) {
    const std::string sensor_name = "pose" + std::to_string(i);
    const std::string config_name = sensor_name + "_config";
    const std::string queue_name = sensor_name + "_queue_size";
    const std::string relative_name = sensor_name + "_relative";
    const std::string differential_name = sensor_name + "_differential";
    const std::string rejection_name = sensor_name + "_rejection_threshold";

    declare_if_needed(sensor_name, rclcpp::ParameterValue(std::string("")));
    std::string topic;
    this->get_parameter(sensor_name, topic);
    if (topic.empty()) {
      break;
    }

    declare_if_needed(config_name, rclcpp::ParameterValue(std::vector<bool>(STATE_SIZE, false)));
    std::vector<bool> config;
    this->get_parameter(config_name, config);

    if (config.size() != STATE_SIZE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid %s size: expected 15, got %zu. Sensor will be skipped.",
        config_name.c_str(), config.size());
      continue;
    }

    declare_if_needed(queue_name, rclcpp::ParameterValue(10));
    declare_if_needed(relative_name, rclcpp::ParameterValue(false));
    declare_if_needed(differential_name, rclcpp::ParameterValue(false));
    declare_if_needed(rejection_name, rclcpp::ParameterValue(std::numeric_limits<double>::max()));

    int queue_size = 10;
    bool relative = false;
    bool differential = false;
    double rejection_threshold = std::numeric_limits<double>::max();
    this->get_parameter(queue_name, queue_size);
    this->get_parameter(relative_name, relative);
    this->get_parameter(differential_name, differential);
    this->get_parameter(rejection_name, rejection_threshold);

    CallbackData cb;
    cb.topic_name_ = topic;
    cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberX, StateMemberYaw);
    cb.update_sum_ = count_updates(cb.update_vector_);
    cb.relative_ = relative;
    cb.differential_ = differential;
    cb.rejection_threshold_ = rejection_threshold;

    auto sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, queue_size))),
      [this, cb](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        this->poseCallback(msg, cb, this->world_frame_id_, this->base_link_frame_id_, false);
      });

    topic_subs_.push_back(sub);
  }

  // Twist sensors
  for (size_t i = 0; i < kMaxIndexedSensors; ++i) {
    const std::string sensor_name = "twist" + std::to_string(i);
    const std::string config_name = sensor_name + "_config";
    const std::string queue_name = sensor_name + "_queue_size";
    const std::string relative_name = sensor_name + "_relative";
    const std::string differential_name = sensor_name + "_differential";
    const std::string rejection_name = sensor_name + "_rejection_threshold";

    declare_if_needed(sensor_name, rclcpp::ParameterValue(std::string("")));
    std::string topic;
    this->get_parameter(sensor_name, topic);
    if (topic.empty()) {
      break;
    }

    declare_if_needed(config_name, rclcpp::ParameterValue(std::vector<bool>(STATE_SIZE, false)));
    std::vector<bool> config;
    this->get_parameter(config_name, config);

    if (config.size() != STATE_SIZE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid %s size: expected 15, got %zu. Sensor will be skipped.",
        config_name.c_str(), config.size());
      continue;
    }

    declare_if_needed(queue_name, rclcpp::ParameterValue(10));
    declare_if_needed(relative_name, rclcpp::ParameterValue(false));
    declare_if_needed(differential_name, rclcpp::ParameterValue(false));
    declare_if_needed(rejection_name, rclcpp::ParameterValue(std::numeric_limits<double>::max()));

    int queue_size = 10;
    bool relative = false;
    bool differential = false;
    double rejection_threshold = std::numeric_limits<double>::max();
    this->get_parameter(queue_name, queue_size);
    this->get_parameter(relative_name, relative);
    this->get_parameter(differential_name, differential);
    this->get_parameter(rejection_name, rejection_threshold);

    CallbackData cb;
    cb.topic_name_ = topic;
    cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberVx, StateMemberVyaw);
    cb.update_sum_ = count_updates(cb.update_vector_);
    cb.relative_ = relative;
    cb.differential_ = differential;
    cb.rejection_threshold_ = rejection_threshold;

    auto sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      topic,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, queue_size))),
      [this, cb](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        this->twistCallback(msg, cb, this->base_link_frame_id_);
      });

    topic_subs_.push_back(sub);
  }

  // IMU sensors
  for (size_t i = 0; i < kMaxIndexedSensors; ++i) {
    const std::string sensor_name = "imu" + std::to_string(i);
    const std::string config_name = sensor_name + "_config";
    const std::string queue_name = sensor_name + "_queue_size";
    const std::string relative_name = sensor_name + "_relative";
    const std::string differential_name = sensor_name + "_differential";
    const std::string rejection_name = sensor_name + "_rejection_threshold";

    declare_if_needed(sensor_name, rclcpp::ParameterValue(std::string("")));
    std::string topic;
    this->get_parameter(sensor_name, topic);
    if (topic.empty()) {
      break;
    }

    declare_if_needed(config_name, rclcpp::ParameterValue(std::vector<bool>(STATE_SIZE, false)));
    std::vector<bool> config;
    this->get_parameter(config_name, config);

    if (config.size() != STATE_SIZE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid %s size: expected 15, got %zu. Sensor will be skipped.",
        config_name.c_str(), config.size());
      continue;
    }

    declare_if_needed(queue_name, rclcpp::ParameterValue(10));
    declare_if_needed(relative_name, rclcpp::ParameterValue(false));
    declare_if_needed(differential_name, rclcpp::ParameterValue(false));
    declare_if_needed(rejection_name, rclcpp::ParameterValue(std::numeric_limits<double>::max()));

    int queue_size = 10;
    bool relative = false;
    bool differential = false;
    double rejection_threshold = std::numeric_limits<double>::max();
    this->get_parameter(queue_name, queue_size);
    this->get_parameter(relative_name, relative);
    this->get_parameter(differential_name, differential);
    this->get_parameter(rejection_name, rejection_threshold);

    CallbackData pose_cb;
    pose_cb.topic_name_ = topic;
    pose_cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberRoll, StateMemberYaw);
    pose_cb.update_sum_ = count_updates(pose_cb.update_vector_);
    pose_cb.relative_ = relative;
    pose_cb.differential_ = differential;
    pose_cb.rejection_threshold_ = rejection_threshold;

    CallbackData twist_cb;
    twist_cb.topic_name_ = topic;
    twist_cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberVroll, StateMemberVyaw);
    twist_cb.update_sum_ = count_updates(twist_cb.update_vector_);
    twist_cb.relative_ = relative;
    twist_cb.differential_ = differential;
    twist_cb.rejection_threshold_ = rejection_threshold;

    CallbackData accel_cb;
    accel_cb.topic_name_ = topic;
    accel_cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberAx, StateMemberAz);
    accel_cb.update_sum_ = count_updates(accel_cb.update_vector_);
    accel_cb.relative_ = relative;
    accel_cb.differential_ = differential;
    accel_cb.rejection_threshold_ = rejection_threshold;

    auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
      topic,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, queue_size))),
      [this, topic, pose_cb, twist_cb, accel_cb](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->imuCallback(msg, topic, pose_cb, twist_cb, accel_cb);
      });

    topic_subs_.push_back(sub);
  }

  // Acceleration sensors (IMU linear acceleration path)
  for (size_t i = 0; i < kMaxIndexedSensors; ++i) {
    const std::string sensor_name = "accel" + std::to_string(i);
    const std::string config_name = sensor_name + "_config";
    const std::string queue_name = sensor_name + "_queue_size";
    const std::string relative_name = sensor_name + "_relative";
    const std::string differential_name = sensor_name + "_differential";
    const std::string rejection_name = sensor_name + "_rejection_threshold";

    declare_if_needed(sensor_name, rclcpp::ParameterValue(std::string("")));
    std::string topic;
    this->get_parameter(sensor_name, topic);
    if (topic.empty()) {
      break;
    }

    declare_if_needed(config_name, rclcpp::ParameterValue(std::vector<bool>(STATE_SIZE, false)));
    std::vector<bool> config;
    this->get_parameter(config_name, config);

    if (config.size() != STATE_SIZE) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid %s size: expected 15, got %zu. Sensor will be skipped.",
        config_name.c_str(), config.size());
      continue;
    }

    declare_if_needed(queue_name, rclcpp::ParameterValue(10));
    declare_if_needed(relative_name, rclcpp::ParameterValue(false));
    declare_if_needed(differential_name, rclcpp::ParameterValue(false));
    declare_if_needed(rejection_name, rclcpp::ParameterValue(std::numeric_limits<double>::max()));

    int queue_size = 10;
    bool relative = false;
    bool differential = false;
    double rejection_threshold = std::numeric_limits<double>::max();
    this->get_parameter(queue_name, queue_size);
    this->get_parameter(relative_name, relative);
    this->get_parameter(differential_name, differential);
    this->get_parameter(rejection_name, rejection_threshold);

    CallbackData cb;
    cb.topic_name_ = topic;
    cb.update_vector_ = make_update_vector(STATE_SIZE, config, StateMemberAx, StateMemberAz);
    cb.update_sum_ = count_updates(cb.update_vector_);
    cb.relative_ = relative;
    cb.differential_ = differential;
    cb.rejection_threshold_ = rejection_threshold;

    auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
      topic,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, queue_size))),
      [this, cb](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->accelerationCallback(msg, cb, this->base_link_frame_id_);
      });

    topic_subs_.push_back(sub);
  }

  enabled_ = !disabled_at_startup_;
}