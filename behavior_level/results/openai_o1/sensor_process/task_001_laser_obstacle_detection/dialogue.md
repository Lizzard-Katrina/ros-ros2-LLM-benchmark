# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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


// Constructor
ScanToScanFilterChain::ScanToScanFilterChain(
  const rclcpp::NodeOptions & options,
  const std::string & ns)
: rclcpp::Node("scan_to_scan_filter_chain", ns, options),
  diagnostic_updater_(this),
  tf_(NULL),
  buffer_(this->get_clock()),
  tf_filter_(NULL),
  filter_chain_("sensor_msgs::msg::LaserScan")
{
  // Heartbeat diagnostics
  diagnostic_updater_.add(heartbeat_diagnostics_);
  diagnostic_updater_.setHardwareID("laser_filters");

  // Configure filter chain
  filter_chain_.configure(
    "",
    this->get_node_logging_interface(), this->get_node_parameters_interface());

  rcl_interfaces::msg::ParameterDescriptor read_only_desc;
  read_only_desc.read_only = true;

  // Declare parameters
  #ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->declare_parameter("lazy_subscription", false, read_only_desc);
  #endif
  this->declare_parameter("tf_message_filter_target_frame", "", read_only_desc);
  this->declare_parameter("tf_message_filter_tolerance", 0.03, read_only_desc);
  this->declare_parameter("scan_filtered_history_depth", 1000);

  // Get parameters
  #ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->get_parameter("lazy_subscription", lazy_subscription_);
  #endif
  this->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame_);
  this->get_parameter("tf_message_filter_tolerance", tf_filter_tolerance_);
  this->get_parameter("scan_filtered_history_depth", scan_filtered_history_depth_);

  //TODO
  // 1. Establish the input 'scan' subscription using SensorDataQoS. 
// 2. Determine the synchronization strategy: If a target frame is provided, 
//    ensure the processing only triggers when TF transforms are ready. 
//    Otherwise, use a direct pass-through to the callback.
// 3. Initialize the output publisher for "scan_filtered".
// 
// CONSTRAINT: You must handle 'lazy_subscription' logic if the platform supports it 
// (RCLCPP_SUPPORTS_MATCHED_CALLBACKS), ensuring the sensor is only active when 
// subscribers are present.
  //ENDOF TODO
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
// TODO: Implement the Processing-to-Output gate. 
// Process the 'msg_in' through the filter chain and broadcast ONLY valid results.
//END OF TODO
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)

----------------------------


# LLM Response

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

#include <memory>
#include <functional>
#include <string>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Constructor
ScanToScanFilterChain::ScanToScanFilterChain(
  const rclcpp::NodeOptions & options,
  const std::string & ns)
: rclcpp::Node("scan_to_scan_filter_chain", ns, options),
  diagnostic_updater_(this),
  tf_(nullptr),
  buffer_(this->get_clock()),
  tf_filter_(nullptr),
  filter_chain_("sensor_msgs::msg::LaserScan")
{
  diagnostic_updater_.add(heartbeat_diagnostics_);
  diagnostic_updater_.setHardwareID("laser_filters");

  filter_chain_.configure(
    "",
    this->get_node_logging_interface(), this->get_node_parameters_interface());

  rcl_interfaces::msg::ParameterDescriptor read_only_desc;
  read_only_desc.read_only = true;

#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->declare_parameter("lazy_subscription", false, read_only_desc);
#endif
  this->declare_parameter("tf_message_filter_target_frame", "", read_only_desc);
  this->declare_parameter("tf_message_filter_tolerance", 0.03, read_only_desc);
  this->declare_parameter("scan_filtered_history_depth", 1000);

#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  this->get_parameter("lazy_subscription", lazy_subscription_);
#endif
  this->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame_);
  this->get_parameter("tf_message_filter_tolerance", tf_filter_tolerance_);
  this->get_parameter("scan_filtered_history_depth", scan_filtered_history_depth_);

  // 1. Establish the input 'scan' subscription using SensorDataQoS.
  // 2. Determine the synchronization strategy.
  // 3. Initialize the output publisher for "scan_filtered", handle lazy subscription if enabled.
  scan_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS().keep_last(scan_filtered_history_depth_));

#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  if (lazy_subscription_) {
    auto manage_sub = [this](rclcpp::MatchedSubscriptionInfo & info) {
      if (info.current_count > 0 && !subscription_) {
        if (!tf_message_filter_target_frame_.empty()) {
          tf_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
          tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            buffer_, tf_message_filter_target_frame_, 10, this->shared_from_this());
          tf_filter_->registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
          tf_filter_->setTolerance(rclcpp::Duration::from_seconds(tf_filter_tolerance_));
        } else {
          subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
        }
      } else if (info.current_count == 0 && subscription_) {
        subscription_.reset();
        tf_filter_.reset();
        tf_.reset();
      }
    };
    scan_filtered_pub_->on_subscription_matched(manage_sub);
  } else {
#endif
    if (!tf_message_filter_target_frame_.empty()) {
      tf_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
      tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        buffer_, tf_message_filter_target_frame_, 10, this->shared_from_this());
      tf_filter_->registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
      tf_filter_->setTolerance(rclcpp::Duration::from_seconds(tf_filter_tolerance_));
    } else {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }
#ifdef RCLCPP_SUPPORTS_MATCHED_CALLBACKS
  }
#endif
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
  // Process the 'msg_in' through the filter chain and broadcast ONLY valid results.
  sensor_msgs::msg::LaserScan msg_out;
  if (filter_chain_.update(*msg_in, msg_out)) {
    scan_filtered_pub_->publish(msg_out);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)