/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "scan_to_scan_filter_chain.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>


// Constructor
ScanToScanFilterChain::ScanToScanFilterChain(
  const rclcpp::NodeOptions & options,
  const std::string & ns)
: rclcpp::Node("scan_to_scan_filter_chain", ns, options),
  tf_(nullptr),
  buffer_(this->get_clock()),
  scan_sub_(),
  tf_filter_(nullptr)
#ifdef HAS_FILTERS
  , filter_chain_("sensor_msgs::msg::LaserScan")
#endif
#ifdef HAS_DIAGNOSTIC_UPDATER
  , diagnostic_updater_(this)
#endif
{
#ifdef HAS_DIAGNOSTIC_UPDATER
  // Heartbeat diagnostics
  diagnostic_updater_.add(heartbeat_diagnostics_);
  diagnostic_updater_.setHardwareID("laser_filters");
#endif

#ifdef HAS_FILTERS
  // Configure filter chain
  filter_chain_.configure(
    "",
    this->get_node_logging_interface(), this->get_node_parameters_interface());
#endif

  rcl_interfaces::msg::ParameterDescriptor read_only_desc;
  read_only_desc.read_only = true;

  // Declare parameters
  this->declare_parameter("tf_message_filter_target_frame", "", read_only_desc);
  this->declare_parameter("tf_message_filter_tolerance", 0.03, read_only_desc);
  this->declare_parameter("scan_filtered_history_depth", 1000);

  // Get parameters
  this->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame_);
  this->get_parameter("tf_message_filter_tolerance", tf_filter_tolerance_);
  this->get_parameter("scan_filtered_history_depth", scan_filtered_history_depth_);

  if (!tf_message_filter_target_frame_.empty()) {
    tf_.reset(new tf2_ros::TransformListener(buffer_, this));
    tf_filter_.reset(
      new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(
        scan_sub_, buffer_, "",
        50, this->get_node_logging_interface(),
        this->get_node_clock_interface()));
    tf_filter_->setTargetFrame(tf_message_filter_target_frame_);
    tf_filter_->setTolerance(std::chrono::duration<double>(tf_filter_tolerance_));

    // Setup tf::MessageFilter generates callback
    tf_filter_->registerCallback(
      std::bind(
        &ScanToScanFilterChain::callback, this,
        std::placeholders::_1));
  } else {
    // Pass through if no tf_message_filter_target_frame
    scan_sub_.registerCallback(
      std::bind(
        &ScanToScanFilterChain::callback, this,
        std::placeholders::_1));
  }

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  output_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_filtered", scan_filtered_history_depth_, pub_options);

  // Use rmw_qos_profile_t for message_filters::Subscriber::subscribe()
  // SensorDataQoS maps to BEST_EFFORT reliability, KEEP_LAST history
  rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
  scan_sub_.subscribe(this, "scan", sensor_qos);
}

// Destructor
ScanToScanFilterChain::~ScanToScanFilterChain()
{
  if (tf_filter_) {
    tf_filter_.reset();
  }
  if (tf_) {
    tf_.reset();
  }
}

// Callback
void ScanToScanFilterChain::callback(
  const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg_in)
{
  // Run the filter chain
#ifdef HAS_FILTERS
  if (filter_chain_.update(*msg_in, msg_)) {
    //only publish result if filter succeeded
    output_pub_->publish(msg_);
  }
#else
  // No filter chain available, pass through
  output_pub_->publish(*msg_in);
#endif
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)