# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "robot_localization/ros_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ekf.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/filter_state.hpp"
#include "robot_localization/filter_utilities.hpp"
#include "robot_localization/ros_filter_utilities.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "robot_localization/srv/toggle_filter_processing.hpp"
#include "robot_localization/ukf.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace robot_localization
{
using namespace std::chrono_literals;

template<typename T>
RosFilter<T>::RosFilter(const rclcpp::NodeOptions & options)
: LifecycleNode(options.arguments()[0], options),
  print_diagnostics_(true),
  publish_acceleration_(false),
  publish_transform_(true),
  reset_on_time_jump_(false),
  smooth_lagged_data_(false),
  toggled_on_(true),
  two_d_mode_(false),
  use_control_(false),
  stamped_control_(true),
  disabled_at_startup_(false),
  enabled_(false),
  permit_corrected_publication_(false),
  dynamic_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  static_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  frequency_(30.0),
  gravitational_acceleration_(9.80665),
  history_length_(0ns),
  sensor_timeout_(0ns),
  latest_control_(),
  process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  initial_estimate_error_covariance_(STATE_SIZE, STATE_SIZE),
  last_diag_time_(0, 0, RCL_ROS_TIME),
  last_published_stamp_(0, 0, RCL_ROS_TIME),
  predict_to_current_time_(false),
  last_set_pose_time_(0, 0, RCL_ROS_TIME),
  latest_control_time_(0, 0, RCL_ROS_TIME),
  tf_timeout_(0ns),
  tf_timeout_odom_bl_(0ns),
  tf_time_offset_(0ns)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  state_variable_names_.push_back("X");
  state_variable_names_.push_back("Y");
  state_variable_names_.push_back("Z");
  state_variable_names_.push_back("ROLL");
  state_variable_names_.push_back("PITCH");
  state_variable_names_.push_back("YAW");
  state_variable_names_.push_back("X_VELOCITY");
  state_variable_names_.push_back("Y_VELOCITY");
  state_variable_names_.push_back("Z_VELOCITY");
  state_variable_names_.push_back("ROLL_VELOCITY");
  state_variable_names_.push_back("PITCH_VELOCITY");
  state_variable_names_.push_back("YAW_VELOCITY");
  state_variable_names_.push_back("X_ACCELERATION");
  state_variable_names_.push_back("Y_ACCELERATION");
  state_variable_names_.push_back("Z_ACCELERATION");

  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("odom_frame", std::string("odom"));
  this->declare_parameter("base_link_frame", std::string("base_link"));
  this->declare_parameter("base_link_frame_output", std::string("base_link"));
  this->declare_parameter("world_frame", std::string("odom"));
  this->declare_parameter("tf_prefix", rclcpp::PARAMETER_STRING);
  this->declare_parameter("print_diagnostics", false);
  this->declare_parameter("debug", false);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("publish_acceleration", false);
  this->declare_parameter("permit_corrected_publication", false);
  this->declare_parameter("predict_to_current_time", false);
  this->declare_parameter("two_d_mode", false);
  this->declare_parameter("smooth_lagged_data", false);
  this->declare_parameter("reset_on_time_jump", false);
  this->declare_parameter("use_control", false);
  this->declare_parameter("stamped_control", true);
  this->declare_parameter("dynamic_process_noise_covariance", false);
  this->declare_parameter("disabled_at_startup", false);
  this->declare_parameter("gravitational_acceleration", 9.80665);
  this->declare_parameter("transform_time_offset", 0.0);
  this->declare_parameter("transform_timeout", 0.0);
  this->declare_parameter("frequency", 30.0);
  this->declare_parameter("sensor_timeout", 1.0 / 30.0);
  this->declare_parameter("history_length", 0.0);
  this->declare_parameter("control_timeout", 0.0);
  this->declare_parameter("debug_out_file", std::string("robot_localization_debug.txt"));
  this->declare_parameter("control_config", rclcpp::PARAMETER_BOOL_ARRAY);
  this->declare_parameter("acceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("acceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("deceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("deceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("initial_state", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("process_noise_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("initial_estimate_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
}

template<typename T>
RosFilter<T>::~RosFilter()
{
  topic_subs_.clear();
  timer_.reset();
  set_pose_sub_.reset();
  control_sub_.reset();
  stamped_control_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  diagnostic_updater_.reset();
  world_transform_broadcaster_.reset();
  set_pose_service_.reset();
  freq_diag_.reset();
  accel_pub_.reset();
  position_pub_.reset();
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosFilter<T>::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(), "[%s]: Transitioning to 'Configured' state. Parameters loaded.",
    get_name());

  // Initialize angular acceleration (no parameters needed)
  angular_acceleration_.setZero();
  angular_acceleration_cov_.setIdentity();
  angular_acceleration_cov_ *= 1e-6;
  last_state_twist_rot_.setZero();

  // Set up diagnostic updater (must be before loadParams)
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(
    shared_from_this());
  diagnostic_updater_->setHardwareID("none");

  // Set up transform broadcaster (must be before loadParams)
  world_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    shared_from_this());

  // Load all parameters - this declares and loads everything
  // After this call, all parameters like print_diagnostics_, frequency_, etc. are available
  loadParams();

  // Now we can safely use parameters that were loaded above
  if (print_diagnostics_) {
    diagnostic_updater_->add(
      "Filter diagnostic updater", this,
      &RosFilter<T>::aggregateDiagnostics);
  }

  // Set up the frequency diagnostic (uses frequency_ parameter)
  min_frequency_ = frequency_ - 2;
  max_frequency_ = frequency_ + 2;
  freq_diag_ =
    std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    "odometry/filtered",
    *diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(
      &min_frequency_,
      &max_frequency_, 0.1, 10));

  // Initialize time tracking
  last_diag_time_ = this->now();
  last_diff_time_ = this->now().seconds();

  // Clear out the transforms
  world_base_link_trans_msg_.transform =
    tf2::toMsg(tf2::Transform::getIdentity());

  RCLCPP_INFO(get_logger(), "[%s]: Node Configured and in Inactive state.", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosFilter<T>::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(), "[%s]: Transitioning to 'Active' state. Publishers and timers started.",
    get_name());

  // Enable filter processing
  enabled_ = true;
  if (!this->get_clock()->started()) {
    RCLCPP_INFO(get_logger(), "Waiting for clock to start...");
    this->get_clock()->wait_until_started();
  }

  // Create lifecycle publishers
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  position_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(
    "odometry/filtered", rclcpp::QoS(10), publisher_options);

  if (publish_acceleration_) {
    accel_pub_ =
      this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "accel/filtered", rclcpp::QoS(10), publisher_options);
  }

  // Activate publishers
  position_pub_->on_activate();
  if (accel_pub_) {
    accel_pub_->on_activate();
  }

  // Start the periodic update timer
  const std::chrono::duration<double> timespan{1.0 / frequency_};
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
    this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
    std::bind(&RosFilter<T>::periodicUpdate, this), this->get_node_base_interface()->get_context());
  auto timer_callback_group = this->get_node_base_interface()->get_default_callback_group();
  this->get_node_timers_interface()->add_timer(timer_, timer_callback_group);

  RCLCPP_INFO(get_logger(), "[%s]: Node Active.", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosFilter<T>::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(get_logger(), "[%s]: Transitioning to 'Inactive' state.", get_name());

  // Reverse the construction order from on_activate to ensure proper cleanup

  // Stop the periodic update timer
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  // Deactivate and destroy publishers to return to pre-activation state
  if (position_pub_) {
    position_pub_->on_deactivate();
    position_pub_.reset();
  }
  if (accel_pub_) {
    accel_pub_->on_deactivate();
    accel_pub_.reset();
  }

  // Disable filter processing and reset internal state
  enabled_ = false;
  reset();

  RCLCPP_WARN(get_logger(), "[%s]: Node Inactive.", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosFilter<T>::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(), "[%s]: Transitioning to 'Unconfigured' state. Resources reset.",
    get_name());

  // Ensure filter is disabled
  enabled_ = false;

  // Clean up diagnostic and transform infrastructure
  diagnostic_updater_.reset();
  world_transform_broadcaster_.reset();
  freq_diag_.reset();

  // Clean up subscribers and services
  set_pose_sub_.reset();
  control_sub_.reset();
  stamped_control_sub_.reset();
  topic_subs_.clear();
  set_pose_service_.reset();
  enable_filter_srv_.reset();
  reset_srv_.reset();
  toggle_filter_processing_srv_.reset();

  // Defensive cleanup for resources that should already be destroyed
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  position_pub_.reset();
  accel_pub_.reset();

  RCLCPP_WARN(get_logger(), "[%s]: Node Cleaned Up. All resources released.", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RosFilter<T>::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(
    get_logger(), "[%s]: Transitioning to 'shutdown' state. Process exiting.",
    get_name());

  // Stop timer if running
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  topic_subs_.clear();
  RCLCPP_WARN(get_logger(), "[%s]: Finalized. System resources released.", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename T>
void RosFilter<T>::reset()
{
  // Get rid of any initial poses (pretend we've never had a measurement)
  initial_measurements_.clear();
  previous_measurements_.clear();
  previous_measurement_covariances_.clear();

  clearMeasurementQueue();

  filter_state_history_.clear();
  measurement_history_.clear();

  angular_acceleration_.setZero();
  angular_acceleration_cov_.setIdentity();
  angular_acceleration_cov_ *= 0.01;

  last_state_twist_rot_.setZero();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  last_set_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_diag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_published_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  last_diff_time_ = this->now().seconds();

  // clear tf buffer to avoid TF_OLD_DATA errors
  tf_buffer_->clear();

  // clear last message timestamp, so older messages will be accepted
  last_message_times_.clear();

  // reset filter to uninitialized state
  filter_.reset();

  // Restore filter parameters that we got from the ROS parameter server
  filter_.setSensorTimeout(sensor_timeout_);
  filter_.setProcessNoiseCovariance(process_noise_covariance_);
  filter_.setEstimateErrorCovariance(initial_estimate_error_covariance_);
}

template<typename T>
void RosFilter<T>::resetSrvCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(
    get_logger(),
    "Received a request to reset filter.");

  reset();
}

template<typename T>
void RosFilter<T>::toggleFilterProcessingCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<
    robot_localization::srv::ToggleFilterProcessing::Request> req,
  const std::shared_ptr<
    robot_localization::srv::ToggleFilterProcessing::Response> resp)
{
  if (req->on == toggled_on_) {
    RCLCPP_WARN(
      get_logger(),
      "Service was called to toggle filter processing but state was already as "
      "requested.");
    resp->status = false;
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Toggling filter measurement filtering to %s.", req->on ? "On" : "Off");
    toggled_on_ = req->on;
    resp->status = true;
  }
}

// @todo: Replace with AccelWithCovarianceStamped
template<typename T>
void RosFilter<T>::accelerationCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const CallbackData & callback_data,
  const std::string & target_frame)
{
  // Ignore sensor data when filter is disabled
  if (!enabled_) {
    return;
  }

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    return;
  }

  const std::string & topic_name = callback_data.topic_name_;

  RF_DEBUG(
    "------ RosFilter<T>::accelerationCallback (" << topic_name <<
      ") ------\n")
  // "Twist message:\n" << *msg);

  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG("Update vector for " << topic_name << " is:\n" << topic_name);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareAcceleration(
        msg, topic_name, target_frame, callback_data.relative_,
        update_vector_corrected, measurement,
        measurement_covariance))
    {
      // Store the measurement. Add an "acceleration" suffix so we know what
      // kind of measurement we're dealing with when we debug the core filter
      // logic.
      enqueueMeasurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG(
        "Enqueued new measurement for " << topic_name <<
          "_acceleration\n");
    } else {
      RF_DEBUG(
        "Did *not* enqueue measurement for " << topic_name <<
          "_acceleration\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        "\n");
  } else {
    // else if (reset_on_time_jump_ && rclcpp::Time::isSimTime())
    //{
    //  reset();
    //}

    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may"
      " indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";

    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, topic_name +
      "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " <<
        topic_name << " is " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        ", current message time is " <<
        filter_utilities::toSec(msg->header.stamp) << ".\n");
  }

  RF_DEBUG(
    "\n----- /RosFilter<T>::accelerationCallback (" << topic_name <<
      ") ------\n");
}

template<typename T>
void RosFilter<T>::controlCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped::SharedPtr twist_stamped_ptr =
    std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped_ptr->twist = *msg;
  twist_stamped_ptr->header.frame_id = base_link_frame_id_;
  twist_stamped_ptr->header.stamp = this->now();
  controlStampedCallback(twist_stamped_ptr);
}

template<typename T>
void RosFilter<T>::controlStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id == base_link_frame_id_ ||
    msg->header.frame_id == "")
  {
    latest_control_(ControlMemberVx) = msg->twist.linear.x;
    latest_control_(ControlMemberVy) = msg->twist.linear.y;
    latest_control_(ControlMemberVz) = msg->twist.linear.z;
    latest_control_(ControlMemberVroll) = msg->twist.angular.x;
    latest_control_(ControlMemberVpitch) = msg->twist.angular.y;
    latest_control_(ControlMemberVyaw) = msg->twist.angular.z;
    latest_control_time_ = msg->header.stamp;

    // Update the filter with this control term
    filter_.setControl(latest_control_, msg->header.stamp);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "Commanded velocities "
      " must be given in the robot's body frame (" << base_link_frame_id_ <<
        "). Message frame was " << msg->header.frame_id);
  }
}

template<typename T>
void RosFilter<T>::enqueueMeasurement(
  const std::string & topic_name, const Eigen::VectorXd & measurement,
  const Eigen::MatrixXd & measurement_covariance,
  const std::vector<bool> & update_vector, const double mahalanobis_thresh,
  const rclcpp::Time & time)
{
  MeasurementPtr meas = MeasurementPtr(new Measurement());

  meas->topic_name_ = topic_name;
  meas->measurement_ = measurement;
  meas->covariance_ = measurement_covariance;
  meas->update_vector_ = update_vector;
  meas->time_ = time;
  meas->mahalanobis_thresh_ = mahalanobis_thresh;
  meas->latest_control_ = latest_control_;
  meas->latest_control_time_ = latest_control_time_;
  measurement_queue_.push(meas);
}

template<typename T>
void RosFilter<T>::forceTwoD(
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  // Force 3D variables to 0 in the measurement
  measurement(StateMemberZ) = 0.0;
  measurement(StateMemberRoll) = 0.0;
  measurement(StateMemberPitch) = 0.0;
  measurement(StateMemberVz) = 0.0;
  measurement(StateMemberVroll) = 0.0;
  measurement(StateMemberVpitch) = 0.0;
  measurement(StateMemberAz) = 0.0;

  // Need to eliminate any off-diagonal covariance values that involve one of our 3D variables
  measurement_covariance.col(StateMemberZ).fill(0.0);
  measurement_covariance.col(StateMemberRoll).fill(0.0);
  measurement_covariance.col(StateMemberPitch).fill(0.0);
  measurement_covariance.col(StateMemberVz).fill(0.0);
  measurement_covariance.col(StateMemberVroll).fill(0.0);
  measurement_covariance.col(StateMemberVpitch).fill(0.0);
  measurement_covariance.col(StateMemberAz).fill(0.0);

  measurement_covariance.row(StateMemberZ).fill(0.0);
  measurement_covariance.row(StateMemberRoll).fill(0.0);
  measurement_covariance.row(StateMemberPitch).fill(0.0);
  measurement_covariance.row(StateMemberVz).fill(0.0);
  measurement_covariance.row(StateMemberVroll).fill(0.0);
  measurement_covariance.row(StateMemberVpitch).fill(0.0);
  measurement_covariance.row(StateMemberAz).fill(0.0);

  // Now set the diagonal covariance values to something small
  measurement_covariance(StateMemberZ, StateMemberZ) = 1e-6;
  measurement_covariance(StateMemberRoll, StateMemberRoll) = 1e-6;
  measurement_covariance(StateMemberPitch, StateMemberPitch) = 1e-6;
  measurement_covariance(StateMemberVz, StateMemberVz) = 1e-6;
  measurement_covariance(StateMemberVroll, StateMemberVroll) = 1e-6;
  measurement_covariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
  measurement_covariance(StateMemberAz, StateMemberAz) = 1e-6;

  // Finally, update the update vector
  update_vector[StateMemberZ] = 1;
  update_vector[StateMemberRoll] = 1;
  update_vector[StateMemberPitch] = 1;
  update_vector[StateMemberVz] = 1;
  update_vector[StateMemberVroll] = 1;
  update_vector[StateMemberVpitch] = 1;
  update_vector[StateMemberAz] = 1;
}

template<typename T>
bool RosFilter<T>::getFilteredOdometryMessage(nav_msgs::msg::Odometry * message)
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = filter_.getState();
    const Eigen::MatrixXd & estimate_error_covariance =
      filter_.getEstimateErrorCovariance();

    // Convert from roll, pitch, and yaw back to quaternion for
    // orientation values
    tf2::Quaternion quat;
    quat.setRPY(
      state(StateMemberRoll), state(StateMemberPitch),
      state(StateMemberYaw));

    // Fill out the message
    message->pose.pose.position.x = state(StateMemberX);
    message->pose.pose.position.y = state(StateMemberY);
    message->pose.pose.position.z = state(StateMemberZ);
    message->pose.pose.orientation.x = quat.x();
    message->pose.pose.orientation.y = quat.y();
    message->pose.pose.orientation.z = quat.z();
    message->pose.pose.orientation.w = quat.w();
    message->twist.twist.linear.x = state(StateMemberVx);
    message->twist.twist.linear.y = state(StateMemberVy);
    message->twist.twist.linear.z = state(StateMemberVz);
    message->twist.twist.angular.x = state(StateMemberVroll);
    message->twist.twist.angular.y = state(StateMemberVpitch);
    message->twist.twist.angular.z = state(StateMemberVyaw);

    // Our covariance matrix layout doesn't quite match
    for (size_t i = 0; i < POSE_SIZE; i++) {
      for (size_t j = 0; j < POSE_SIZE; j++) {
        message->pose.covariance[POSE_SIZE * i + j] =
          estimate_error_covariance(i, j);
      }
    }

    // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a
    // few cycles to be meticulous and not index a twist covariance array on the
    // size of a pose covariance array
    for (size_t i = 0; i < TWIST_SIZE; i++) {
      for (size_t j = 0; j < TWIST_SIZE; j++) {
        message->twist.covariance[TWIST_SIZE * i + j] =
          estimate_error_covariance(
          i + POSITION_V_OFFSET,
          j + POSITION_V_OFFSET);
      }
    }

    message->header.stamp = filter_.getLastMeasurementTime();
    message->header.frame_id = world_frame_id_;
    message->child_frame_id = base_link_output_frame_id_;
  }

  return filter_.getInitializedStatus();
}

template<typename T>
bool RosFilter<T>::getFilteredAccelMessage(
  geometry_msgs::msg::AccelWithCovarianceStamped * message)
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = filter_.getState();
    const Eigen::MatrixXd & estimate_error_covariance =
      filter_.getEstimateErrorCovariance();

    //! Fill out the accel_msg
    message->accel.accel.linear.x = state(StateMemberAx);
    message->accel.accel.linear.y = state(StateMemberAy);
    message->accel.accel.linear.z = state(StateMemberAz);
    message->accel.accel.angular.x = angular_acceleration_.x();
    message->accel.accel.angular.y = angular_acceleration_.y();
    message->accel.accel.angular.z = angular_acceleration_.z();

    // Fill the covariance (only the left-upper matrix since we are not
    // estimating the rotational accelerations arround the axes
    for (size_t i = 0; i < ACCELERATION_SIZE; i++) {
      for (size_t j = 0; j < ACCELERATION_SIZE; j++) {
        // We use the POSE_SIZE since the accel cov matrix of ROS is 6x6
        message->accel.covariance[POSE_SIZE * i + j] = estimate_error_covariance(
          i + POSITION_A_OFFSET, j + POSITION_A_OFFSET);
      }
    }
    for (size_t i = ACCELERATION_SIZE; i < POSE_SIZE; i++) {
      for (size_t j = ACCELERATION_SIZE; j < POSE_SIZE; j++) {
        // fill out the angular portion. We assume the linear and angular portions are independent.
        message->accel.covariance[POSE_SIZE * i + j] =
          angular_acceleration_cov_(i - ACCELERATION_SIZE, j - ACCELERATION_SIZE);
      }
    }

    // Fill header information
    message->header.stamp = rclcpp::Time(filter_.getLastMeasurementTime());
    message->header.frame_id = base_link_output_frame_id_;
  }

  return filter_.getInitializedStatus();
}

template<typename T>
void RosFilter<T>::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const std::string & topic_name,
  const CallbackData & pose_callback_data,
  const CallbackData & twist_callback_data,
  const CallbackData & accel_callback_data)
{
  // Ignore sensor data when filter is disabled
  if (!enabled_) {
    return;
  }

  RF_DEBUG(
    "------ RosFilter<T>::imuCallback (" <<
      topic_name << ") ------\n")         // << "IMU message:\n" << *msg);

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp equal to or"
      " before the last filter reset, " << "this message will be ignored. This may"
      "indicate an empty or bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);


    RF_DEBUG(
      "Received message that preceded the most recent pose reset. "
      "Ignoring...");

    return;
  }

  // As with the odometry message, we can separate out the pose- and
  // twist-related variables in the IMU message and pass them to the pose and
  // twist callbacks (filters)
  if (pose_callback_data.update_sum_ > 0) {
    // Per the IMU message specification, if the IMU does not provide
    // orientation, then its first covariance value should be set to -1, and we
    // should ignore that portion of the message. robot_localization allows
    // users to explicitly ignore data using its parameters, but we should also
    // be compliant with message specs.
    if (std::abs(msg->orientation_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "orientation. "
        "Ignoring orientation...");
    } else {
      // Extract the pose (orientation) data, pass it to its filter
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos_ptr =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pos_ptr->header = msg->header;
      pos_ptr->pose.pose.orientation = msg->orientation;

      // Copy the covariance for roll, pitch, and yaw
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          pos_ptr->pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->orientation_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      // IMU data gets handled a bit differently, since the message is ambiguous
      // and has only a single frame_id, even though the data in it is reported
      // in two different frames. As we assume users will specify a base_link to
      // imu transform, we make the target and child frame base_link_frame_id_ and
      // tell the poseCallback that it is working with IMU data. This will cause
      // it to apply different logic to the data.
      poseCallback(
        pos_ptr, pose_callback_data, base_link_frame_id_,
        base_link_frame_id_, true);
    }
  }

  if (twist_callback_data.update_sum_ > 0) {
    // Ignore rotational velocity if the first covariance value is -1
    if (std::abs(msg->angular_velocity_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "angular "
        "velocity. Ignoring angular velocity...");
    } else {
      // Repeat for velocity
      geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
        std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
      twist_ptr->header = msg->header;
      twist_ptr->twist.twist.angular = msg->angular_velocity;

      // Copy the covariance
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          twist_ptr->twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      twistCallback(twist_ptr, twist_callback_data, base_link_frame_id_);
    }
  }

  if (accel_callback_data.update_sum_ > 0) {
    // Ignore linear acceleration if the first covariance value is -1
    if (std::abs(msg->linear_acceleration_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "linear "
        "acceleration. Ignoring linear acceleration...");
    } else {
      // Pass the message on
      accelerationCallback(msg, accel_callback_data, base_link_frame_id_);
    }
  }

  RF_DEBUG("\n----- /RosFilter<T>::imuCallback (" << topic_name << ") ------\n");
}

template<typename T>
void RosFilter<T>::integrateMeasurements(const rclcpp::Time & current_time)
{
  RF_DEBUG(
    "------ RosFilter<T>::integrateMeasurements ------\n\n"
    "Integration time is " <<
      std::setprecision(20) << filter_utilities::toSec(current_time) <<
      "\n" <<
      measurement_queue_.size() << " measurements in queue.\n");

  bool predict_to_current_time = predict_to_current_time_;

  // If we have any measurements in the queue, process them
  if (!measurement_queue_.empty()) {
    // Check if the first measurement we're going to process is older than the
    // filter's last measurement. This means we have received an out-of-sequence
    // message (one with an old timestamp), and we need to revert both the
    // filter state and measurement queue to the first state that preceded the
    // time stamp of our first measurement.
    const MeasurementPtr & first_measurement = measurement_queue_.top();
    int restored_measurement_count = 0;
    if (smooth_lagged_data_ &&
      first_measurement->time_ < filter_.getLastMeasurementTime())
    {
      RF_DEBUG(
        "Received a measurement that was " <<
          filter_utilities::toSec(
          filter_.getLastMeasurementTime() -
          first_measurement->time_) <<
          " seconds in the past. Reverting filter state and "
          "measurement queue...");

      int original_count = static_cast<int>(measurement_queue_.size());
      const rclcpp::Time first_measurement_time = first_measurement->time_;
      const std::string first_measurement_topic =
        first_measurement->topic_name_;
      // revertTo may invalidate first_measurement
      if (!revertTo(first_measurement_time - rclcpp::Duration(1ns))) {
        RF_DEBUG(
          "ERROR: history interval is too small to revert to time " <<
            filter_utilities::toSec(first_measurement_time) << "\n");
        // ROS_WARN_STREAM_DELAYED_THROTTLE(history_length_,
        //   "Received old measurement for topic " << first_measurement_topic <<
        //   ", but history interval is insufficiently sized. "
        //   "Measurement time is " << std::setprecision(20) <<
        //   first_measurement_time <<
        //   ", current time is " << current_time <<
        //   ", history length is " << history_length_ << ".");
        restored_measurement_count = 0;
      }

      restored_measurement_count =
        static_cast<int>(measurement_queue_.size()) - original_count;
    }

    while (!measurement_queue_.empty() && rclcpp::ok()) {
      MeasurementPtr measurement = measurement_queue_.top();

      // If we've reached a measurement that has a time later than now, it
      // should wait until a future iteration. Since measurements are stored in
      // a priority queue, all remaining measurements will be in the future.
      if (current_time < measurement->time_) {
        break;
      }

      measurement_queue_.pop();

      // When we receive control messages, we call this directly in the control
      // callback. However, we also associate a control with each sensor message
      // so that we can support lagged smoothing. As we cannot guarantee that
      // the new control callback will fire before a new measurement, we should
      // only perform this operation if we are processing messages from the
      // history. Otherwise, we may get a new measurement, store the "old"
      // latest control, then receive a control, call setControl, and then
      // overwrite that value with this one (i.e., with the "old" control we
      // associated with the measurement).
      if (use_control_ && restored_measurement_count > 0) {
        filter_.setControl(
          measurement->latest_control_,
          measurement->latest_control_time_);
        restored_measurement_count--;
      }

      auto previous_state = filter_.getState();
      auto previous_covar = filter_.getEstimateErrorCovariance();
      auto last_measurement_time = filter_.getLastMeasurementTime();

      // This will call predict and, if necessary, correct
      filter_.processMeasurement(*(measurement.get()));

      // Store old states and measurements if we're smoothing
      if (smooth_lagged_data_) {
        // Invariant still holds: measurementHistoryDeque_.back().time_ <
        // measurement_queue_.top().time_
        measurement_history_.push_back(measurement);

        // We should only save the filter state once per unique timstamp
        if (measurement_queue_.empty() ||
          measurement_queue_.top()->time_ !=
          filter_.getLastMeasurementTime())
        {
          saveFilterState(filter_);
        }
      }
    }
  } else if (filter_.getInitializedStatus()) {
    // In the event that we don't get any measurements for a long time,
    // we still need to continue to estimate our state. Therefore, we
    // should project the state forward here.
    rclcpp::Duration last_update_delta =
      current_time - filter_.getLastMeasurementTime();

    // If we get a large delta, then continuously predict until
    if (last_update_delta >= filter_.getSensorTimeout()) {
      predict_to_current_time = true;

      RF_DEBUG(
        "Sensor timeout! Last measurement time was " <<
          filter_utilities::toSec(filter_.getLastMeasurementTime()) <<
          ", current time is " << filter_utilities::toSec(current_time) <<
          ", delta is " << filter_utilities::toSec(last_update_delta) <<
          "\n");
    }
  } else {
    RF_DEBUG("Filter not yet initialized.\n");
  }

  if (filter_.getInitializedStatus() && predict_to_current_time) {
    rclcpp::Duration last_update_delta =
      current_time - filter_.getLastMeasurementTime();

    filter_.validateDelta(last_update_delta);
    filter_.predict(current_time, last_update_delta);

    // Update the last measurement time and last update time
    filter_.setLastMeasurementTime(
      filter_.getLastMeasurementTime() +
      last_update_delta);
  }

  RF_DEBUG("\n----- /RosFilter<T>::integrateMeasurements ------\n");
}

template<typename T>
void RosFilter<T>::differentiateMeasurements(const rclcpp::Time & current_time)
{
  if (filter_.getInitializedStatus()) {
    const double time_now = filter_utilities::toSec(current_time);
    const double dt = time_now - last_diff_time_;
    const Eigen::VectorXd & state = filter_.getState();
    tf2::Vector3 new_state_twist_rot(
      state(StateMemberVroll),
      state(StateMemberVpitch),
      state(StateMemberVyaw));
    angular_acceleration_ = (new_state_twist_rot - last_state_twist_rot_) / dt;
    const Eigen::MatrixXd & cov = filter_.getEstimateErrorCovariance();
    for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
      for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
        angular_acceleration_cov_(i, j) =
          cov(i + ORIENTATION_V_OFFSET, j + ORIENTATION_V_OFFSET) * 2. /
          ( dt * dt );
      }
    }
    last_state_twist_rot_ = new_state_twist_rot;
    last_diff_time_ = time_now;
  }
}

template<typename T>
void RosFilter<T>::loadParams()
{
/**
   * TODO: Task 010 - Migrate robot_localization parameter loading to ROS 2.
   * * 1. Core Logic:
   * - Implement the logic to discover and configure sensors (odom, pose, twist, imu, accel).
   * - Use a loop or systematic approach to check for parameters like "odom0", "odom1", etc.
   * - For each detected sensor, you MUST declare and retrieve its configuration matrix.
   * * 2. Strict Naming & Type Constraints (Required for Oracle Tests):
   * - Topic parameter: Fetch using `this->get_parameter` or `node_->get_parameter`.
   * - Config matrix parameter: MUST be named `<base_type><index>_config` (e.g., "odom0_config").
   * - Config matrix type: MUST be declared as `std::vector<bool>`.
   * - Validation: You MUST verify the config vector has exactly 15 elements. 
   * If invalid, log a warning using `RCLCPP_WARN(this->get_logger(), ...)`.
   * * 3. Architectural Requirements:
   * - Use ROS 2 C++ API exclusively (no ros::NodeHandle).
   * - Parameters MUST be declared before being accessed.
   * - Ensure the internal 'topic_subs_' or equivalent structure is populated with 'CallbackData'.
   * END OF TODO   
*/
}


----------------------------


# LLM Response

```cpp
template<typename T>
void RosFilter<T>::loadParams()
{
  // Sensor configuration parameter names
  const std::vector<std::string> sensor_types = {"odom", "pose", "twist", "imu", "accel"};
  
  // First pass: declare all sensor configuration parameters
  for (const auto& sensor_type : sensor_types) {
    for (int i = 0; i < 100; i++) {  // Check up to 100 sensors of each type
      std::string topic_param = sensor_type + std::to_string(i);
      std::string config_param = topic_param + "_config";
      
      // Declare the config parameter as bool array (required for ROS 2 parameter server)
      this->declare_parameter(config_param, rclcpp::PARAMETER_BOOL_ARRAY);
    }
  }
  
  // Get sensor parameters and populate topic_subs_
  for (const auto& sensor_type : sensor_types) {
    for (int i = 0; i < 100; i++) {
      std::string topic_param = sensor_type + std::to_string(i);
      
      // Get the topic name for this sensor
      std::string topic_name;
      if (!this->get_parameter(topic_param, topic_name) || topic_name.empty()) {
        continue;
      }
      
      // Get the configuration matrix - MUST be named <base_type><index>_config
      std::string config_param = topic_param + "_config";
      std::vector<bool> config_vec;
      if (!this->get_parameter(config_param, config_vec)) {
        RCLCPP_WARN(
          get_logger(),
          "%s sensor specified but %s configuration parameter not found, skipping.",
          topic_param.c_str(), config_param.c_str());
        continue;
      }
      
      // Validate config vector has exactly 15 elements
      if (config_vec.size() != STATE_SIZE) {
        RCLCPP_WARN(
          get_logger(),
          "%s configuration parameter must have %d elements, but has %zu elements. "
          "Sensor %s will be skipped.",
          config_param.c_str(), STATE_SIZE, config_vec.size(), topic_param.c_str());
        continue;
      }
      
      // Build update vector from config
      std::vector<bool> update_vector(config_vec.begin(), config_vec.end());
      
      // Calculate update sum
      int update_sum = 0;
      for (size_t j = 0; j < update_vector.size(); ++j) {
        if (update_vector[j]) {
          update_sum++;
        }
      }
      
      // Only process if at least one element is true
      if (update_sum == 0) {
        continue;
      }
      
      // Create rejection threshold parameter name
      std::string rejection_param = topic_param + "_rejection_threshold";
      this->declare_parameter(rejection_param, 0.0);
      double rejection_threshold;
      this->get_parameter(rejection_param, rejection_threshold);
      
      // Create relative parameter name
      std::string relative_param = topic_param + "_relative";
      this->declare_parameter(relative_param, false);
      bool relative;
      this->get_parameter(relative_param, relative);
      
      // Create CallbackData and add to topic_subs_
      CallbackData callback_data;
      callback_data.topic_name_ = topic_name;
      callback_data.update_vector_ = update_vector;
      callback_data.update_sum_ = update_sum;
      callback_data.rejection_threshold_ = rejection_threshold;
      callback_data.relative_ = relative;
      
      topic_subs_[topic_name] = callback_data;
    }
  }
  
  // Get general parameters
  this->get_parameter("sensor_timeout", sensor_timeout_);
  sensor_timeout_ = rclcpp::Duration::from_seconds(sensor_timeout_);
  
  this->get_parameter("transform_timeout", tf_timeout_);
  tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_);
  
  this->get_parameter("transform_time_offset", tf_time_offset_);
  tf_time_offset_ = rclcpp::Duration::from_seconds(tf_time_offset_);
  
  this->get_parameter("history_length", history_length_);
  history_length_ = rclcpp::Duration::from_seconds(history_length_);
  
  this->get_parameter("control_timeout", control_timeout_);
  control_timeout_ = rclcpp::Duration::from_seconds(control_timeout_);
  
  // Process noise covariance
  std::vector<double> process_noise_vec;
  if (this->get_parameter("process_noise_covariance", process_noise_vec)) {
    if (process_noise_vec.size() == STATE_SIZE * STATE_SIZE) {
      for (size_t i = 0; i < STATE_SIZE; ++i) {
        for (size_t j = 0; j < STATE_SIZE; ++j) {
          process_noise_covariance_(i, j) = process_noise_vec[i * STATE_SIZE + j];
        }
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "process_noise_covariance should have %d elements, but has %zu. Using default.",
        STATE_SIZE * STATE_SIZE, process_noise_vec.size());
    }
  }
  
  // Initial estimate error covariance
  std::vector<double> initial_cov_vec;
  if (this->get_parameter("initial_estimate_covariance", initial_cov_vec)) {
    if (initial_cov_vec.size() == STATE_SIZE * STATE_SIZE) {
      for (size_t i = 0; i < STATE_SIZE; ++i) {
        for (size_t j = 0; j < STATE_SIZE; ++j) {
          initial_estimate_error_covariance_(i, j) = initial_cov_vec[i * STATE_SIZE + j];
        }
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "initial_estimate_covariance should have %d elements, but has %zu. Using default.",
        STATE_SIZE * STATE_SIZE, initial_cov_vec.size());
    }
  }
  
  // Acceleration limits and gains
  std::vector<double> accel_limits;
  if (this->get_parameter("acceleration_limits", accel_limits) && accel_limits.size() == 6) {
    acceleration_limits_ = Eigen::Matrix<double, 6, 1>::Map(accel_limits.data());
  }
  
  std::vector<double> accel_gains;
  if (this->get_parameter("acceleration_gains", accel_gains) && accel_gains.size() == 6) {
    acceleration_gains_ = Eigen::Matrix<double, 6, 1>::Map(accel_gains.data());
  }
  
  std::vector<double> decel_limits;
  if (this->get_parameter("deceleration_limits", decel_limits) && decel_limits.size() == 6) {
    deceleration_limits_ = Eigen::Matrix<double, 6, 1>::Map(decel_limits.data());
  }
  
  std::vector<double> decel_gains;
  if (this->get_parameter("deceleration_gains", decel_gains) && decel_gains.size() == 6) {
    deceleration_gains_ = Eigen::Matrix<double, 6, 1>::Map(decel_gains.data());
  }
  
  // Control configuration
  std::vector<int> control_config_vec;
  if (this->get_parameter("control_config", control_config_vec)) {
    if (control_config_vec.size() == 6) {
      for (size_t i = 0; i < 6; ++i) {
        control_config_[i] = control_config_vec[i] != 0;
      }
    }
  }
}
```