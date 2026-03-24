namespace diff_drive_controller{

  DiffDriveController::DiffDriveController()
    : open_loop_(false)
    , command_struct_()
    , wheel_separation_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_multiplier_(1.0)
    , left_wheel_radius_multiplier_(1.0)
    , right_wheel_radius_multiplier_(1.0)
    , cmd_vel_timeout_(0.5)
    , allow_multiple_cmd_vel_publishers_(true)
    , base_frame_id_("base_link")
    , odom_frame_id_("odom")
    , enable_odom_tf_(true)
    , wheel_joints_size_(0)
    , publish_cmd_(false)
    , publish_wheel_joint_controller_state_(false)
  {
  }

  bool DiffDriveController::init(hardware_interface::VelocityJointInterface* hw,
            rclcpp::Node::SharedPtr root_nh,
            rclcpp::Node::SharedPtr controller_nh)
  {
    const std::string complete_ns = controller_nh->get_namespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_wheel_names, right_wheel_names;
    if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
        !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
    {
      return false;
    }

    if (left_wheel_names.size() != right_wheel_names.size())
    {
      RCLCPP_ERROR(controller_nh->get_logger(),
          "#left wheels (%zu) != #right wheels (%zu).",
          left_wheel_names.size(), right_wheel_names.size());
      return false;
    }
    else
    {
      wheel_joints_size_ = left_wheel_names.size();

      left_wheel_joints_.resize(wheel_joints_size_);
      right_wheel_joints_.resize(wheel_joints_size_);
    }

    // Odometry related:
    double publish_rate;
    publish_rate = controller_nh->declare_parameter<double>("publish_rate", 50.0);
    RCLCPP_INFO(controller_nh->get_logger(), "Controller state will be published at %.2f Hz.", publish_rate);
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);

    //TODO:
    //Declare and retrieve 'open_loop' (bool, default: false), 'velocity_rolling_window_size' (int, default: 10), 
    //and 'cmd_vel_timeout' (double, default: 0.5).
    //Assign values to class members: open_loop_, cmd_vel_timeout_, and call odometry_.setVelocityRollingWindowSize().
    //Log the 'cmd_vel_timeout' value using the ROS 2 logging API.
    open_loop_ = controller_nh->declare_parameter<bool>("open_loop", false);
    int velocity_rolling_window_size = controller_nh->declare_parameter<int>("velocity_rolling_window_size", 10);
    cmd_vel_timeout_ = controller_nh->declare_parameter<double>("cmd_vel_timeout", 0.5);

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    RCLCPP_INFO(controller_nh->get_logger(), "cmd_vel timeout set to %.3f", cmd_vel_timeout_);
    //END OF TODO

    base_frame_id_ = controller_nh->declare_parameter<std::string>("base_frame_id", base_frame_id_);
    RCLCPP_INFO(controller_nh->get_logger(), "Base frame_id set to %s", base_frame_id_.c_str());

    odom_frame_id_ = controller_nh->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
    RCLCPP_INFO(controller_nh->get_logger(), "Odometry frame_id set to %s", odom_frame_id_.c_str());

    enable_odom_tf_ = controller_nh->declare_parameter<bool>("enable_odom_tf", enable_odom_tf_);
    RCLCPP_INFO(controller_nh->get_logger(), "Publishing to tf is %s", enable_odom_tf_ ? "enabled" : "disabled");

    // Velocity and acceleration limits:
    limiter_lin_.has_velocity_limits     = controller_nh->declare_parameter<bool>("linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits);
    limiter_lin_.has_acceleration_limits = controller_nh->declare_parameter<bool>("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits);
    limiter_lin_.has_jerk_limits         = controller_nh->declare_parameter<bool>("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits);
    limiter_lin_.max_velocity             = controller_nh->declare_parameter<double>("linear/x/max_velocity", limiter_lin_.max_velocity);
    limiter_lin_.min_velocity             = controller_nh->declare_parameter<double>("linear/x/min_velocity", limiter_lin_.min_velocity);
    limiter_lin_.max_acceleration         = controller_nh->declare_parameter<double>("linear/x/max_acceleration", limiter_lin_.max_acceleration);
    limiter_lin_.min_acceleration         = controller_nh->declare_parameter<double>("linear/x/min_acceleration", limiter_lin_.min_acceleration);
    limiter_lin_.max_jerk                 = controller_nh->declare_parameter<double>("linear/x/max_jerk", limiter_lin_.max_jerk);
    limiter_lin_.min_jerk                 = controller_nh->declare_parameter<double>("linear/x/min_jerk", limiter_lin_.min_jerk);

    limiter_ang_.has_velocity_limits     = controller_nh->declare_parameter<bool>("angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits);
    limiter_ang_.has_acceleration_limits = controller_nh->declare_parameter<bool>("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits);
    limiter_ang_.has_jerk_limits         = controller_nh->declare_parameter<bool>("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits);
    limiter_ang_.max_velocity             = controller_nh->declare_parameter<double>("angular/z/max_velocity", limiter_ang_.max_velocity);
    limiter_ang_.min_velocity             = controller_nh->declare_parameter<double>("angular/z/min_velocity", limiter_ang_.min_velocity);
    limiter_ang_.max_acceleration         = controller_nh->declare_parameter<double>("angular/z/max_acceleration", limiter_ang_.max_acceleration);
    limiter_ang_.min_acceleration         = controller_nh->declare_parameter<double>("angular/z/min_acceleration", limiter_ang_.min_acceleration);
    limiter_ang_.max_jerk                 = controller_nh->declare_parameter<double>("angular/z/max_jerk", limiter_ang_.max_jerk);
    limiter_ang_.min_jerk                 = controller_nh->declare_parameter<double>("angular/z/min_jerk", limiter_ang_.min_jerk);

    publish_cmd_ = controller_nh->declare_parameter<bool>("publish_cmd", publish_cmd_);
    publish_wheel_joint_controller_state_ = controller_nh->declare_parameter<bool>("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_);

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh->has_parameter("wheel_separation");
    bool lookup_wheel_radius = !controller_nh->has_parameter("wheel_radius");

    if (!setOdomParamsFromUrdf(root_nh,
                              left_wheel_names[0],
                              right_wheel_names[0],
                              lookup_wheel_separation,
                              lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, lwr, rwr);
    RCLCPP_INFO(controller_nh->get_logger(),
                          "Odometry params : wheel separation %.6f, left wheel radius %.6f, right wheel radius %.6f",
                          ws, lwr, rwr);

    if (publish_cmd_)
    {
      cmd_vel_pub_ = controller_nh->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_out", rclcpp::QoS(100));
    }

    // Wheel joint controller state:
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_ = controller_nh->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("wheel_joint_controller_state", rclcpp::QoS(100));

      const size_t num_wheels = wheel_joints_size_ * 2;

      controller_state_msg_.joint_names.resize(num_wheels);

      controller_state_msg_.desired.positions.resize(num_wheels);
      controller_state_msg_.desired.velocities.resize(num_wheels);
      controller_state_msg_.desired.accelerations.resize(num_wheels);
      controller_state_msg_.desired.effort.resize(num_wheels);

      controller_state_msg_.actual.positions.resize(num_wheels);
      controller_state_msg_.actual.velocities.resize(num_wheels);
      controller_state_msg_.actual.accelerations.resize(num_wheels);
      controller_state_msg_.actual.effort.resize(num_wheels);

      controller_state_msg_.error.positions.resize(num_wheels);
      controller_state_msg_.error.velocities.resize(num_wheels);
      controller_state_msg_.error.accelerations.resize(num_wheels);
      controller_state_msg_.error.effort.resize(num_wheels);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        controller_state_msg_.joint_names[i] = left_wheel_names[i];
        controller_state_msg_.joint_names[i + wheel_joints_size_] = right_wheel_names[i];
      }

      vel_left_previous_.resize(wheel_joints_size_, 0.0);
      vel_right_previous_.resize(wheel_joints_size_, 0.0);
    }

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      RCLCPP_INFO(controller_nh->get_logger(),
                            "Adding left wheel with joint name: %s and right wheel with joint name: %s",
                            left_wheel_names[i].c_str(), right_wheel_names[i].c_str());
      left_wheel_joints_[i] = hw->get_handle(left_wheel_names[i]);  // throws on failure
      right_wheel_joints_[i] = hw->get_handle(right_wheel_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Initialize dynamic parameters
    DynamicParams dynamic_params;
    dynamic_params.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    dynamic_params.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
    dynamic_params.wheel_separation_multiplier   = wheel_separation_multiplier_;

    dynamic_params.publish_rate = publish_rate;
    dynamic_params.enable_odom_tf = enable_odom_tf_;

    dynamic_params_.writeFromNonRT(dynamic_params);

    // Initialize dynamic_reconfigure server
    DiffDriveControllerConfig config;
    config.left_wheel_radius_multiplier  = left_wheel_radius_multiplier_;
    config.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
    config.wheel_separation_multiplier   = wheel_separation_multiplier_;

    config.publish_rate = publish_rate;
    config.enable_odom_tf = enable_odom_tf_;

    dyn_reconf_server_ = std::make_shared<ReconfigureServer>(dyn_reconf_server_mutex_, controller_nh);

    // Update parameters
    {
      std::lock_guard<std::mutex> lock(dyn_reconf_server_mutex_);
      dyn_reconf_server_->updateConfig(config);
    }

    dyn_reconf_server_->setCallback(
        std::bind(&DiffDriveController::reconfCallback, this, std::placeholders::_1, std::placeholders::_2));

    return true;
  }

  void DiffDriveController::setOdomPubFields(rclcpp::Node::SharedPtr root_nh, rclcpp::Node::SharedPtr controller_nh)
  {
    /* * TODO (Task 3):
 * 1. Declare and retrieve 'pose_covariance_diagonal' and 'twist_covariance_diagonal' as double arrays.
 * 2. Validate that each retrieved vector has exactly 6 elements.
 * 3. If validation fails, throw a std::invalid_argument with the message "diagonal size must be 6".
 * 4. Store the vectors into local std::vector<double> variables for later mapping.
 * * [Style Constraints]:
 * - Must use std::vector<double> as the underlying data structure.
 * - Use node->declare_parameter<std::vector<double>>().
 * - Use explicit size() check and throw statement.
 */
    std::vector<double> pose_cov_list = controller_nh->declare_parameter<std::vector<double>>("pose_covariance_diagonal", std::vector<double>(6, 0.0));
    std::vector<double> twist_cov_list = controller_nh->declare_parameter<std::vector<double>>("twist_covariance_diagonal", std::vector<double>(6, 0.0));

    if (pose_cov_list.size() != 6)
    {
      throw std::invalid_argument("pose_covariance_diagonal size must be 6");
    }
    if (twist_cov_list.size() != 6)
    {
      throw std::invalid_argument("twist_covariance_diagonal size must be 6");
    }
    //END of TODO

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_ = controller_nh->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(100));
    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = base_frame_id_;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.covariance = {
        pose_cov_list[0], 0., 0., 0., 0., 0.,
        0., pose_cov_list[1], 0., 0., 0., 0.,
        0., 0., pose_cov_list[2], 0., 0., 0.,
        0., 0., 0., pose_cov_list[3], 0., 0.,
        0., 0., 0., 0., pose_cov_list[4], 0.,
        0., 0., 0., 0., 0., pose_cov_list[5] };
    odom_msg_.twist.twist.linear.y  = 0;
    odom_msg_.twist.twist.linear.z  = 0;
    odom_msg_.twist.twist.angular.x = 0;
    odom_msg_.twist.twist.angular.y = 0;
    odom_msg_.twist.covariance = {
        twist_cov_list[0], 0., 0., 0., 0., 0.,
        0., twist_cov_list[1], 0., 0., 0., 0.,
        0., 0., twist_cov_list[2], 0., 0., 0.,
        0., 0., 0., twist_cov_list[3], 0., 0.,
        0., 0., 0., 0., twist_cov_list[4], 0.,
        0., 0., 0., 0., 0., twist_cov_list[5] };

    tf_odom_pub_ = root_nh->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    tf_odom_msg_.transforms.resize(1);
    tf_odom_msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_msg_.transforms[0].header.frame_id = odom_frame_id_;
  }

} // namespace diff_drive_controller