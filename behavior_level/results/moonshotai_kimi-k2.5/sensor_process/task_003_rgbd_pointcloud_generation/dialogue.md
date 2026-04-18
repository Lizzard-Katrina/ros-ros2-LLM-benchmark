# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.hpp"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depth_image_proc
{

PointCloudXyzrgbNode::PointCloudXyzrgbNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudXyzrgbNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  // value used for invalid points for pcd conversion
  invalid_depth_ = this->declare_parameter<double>("invalid_depth", 0.0);

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync) {
    exact_sync_ = std::make_shared<ExactSynchronizer>(
      ExactSyncPolicy(queue_size),
      sub_depth_,
      sub_rgb_,
      sub_info_);
    exact_sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  } else {
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
    sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  }

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo & s)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (s.current_count == 0) {
        sub_depth_.unsubscribe();
        sub_rgb_.unsubscribe();
        sub_info_.unsubscribe();
      } else if (!sub_depth_.getSubscriber()) {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string depth_topic =
          node_base->resolve_topic_or_service_name("depth_registered/image_rect", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_rect_color", false);
        // Allow also remapping camera_info to something different than default
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
          image_transport::getCameraInfoTopic(rgb_topic), false);

        // parameter for depth_image_transport hint
        image_transport::TransportHints depth_hints(*this,
          "raw", "depth_image_transport");

        rclcpp::SubscriptionOptions sub_opts;
        // Update the subscription options to allow reconfigurable qos settings.
        sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
          {
            // Here all policies that are desired to be reconfigurable are listed.
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability,
          }};

        // depth image can use different transport.(e.g. compressedDepth)
        sub_depth_.subscribe(
          *this, depth_topic,
          depth_hints.getTransport(), rclcpp::SystemDefaultsQoS(), sub_opts);

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints{*this};
        sub_rgb_.subscribe(
          *this,
          rgb_topic,
          hints.getTransport(),
          rclcpp::SystemDefaultsQoS(), sub_opts);
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };
  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SystemDefaultsQoS(),
      pub_options);
}

void PointCloudXyzrgbNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,  // 10 seconds
      "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);
  //TODO
  // FUNCTIONAL REQUIREMENTS:
// 1. Resolution Scaling: If Depth and RGB resolutions differ, resize RGB to match Depth.
//    MANDATORY: You must manually scale fx, fy, cx, and cy in the camera model by the 
//    calculated ratio. Do NOT rely on 'setSize' alone as it does not scale intrinsics.
// 2. Encoding Logic: Support RGB8, BGR8, and MONO8. 
//    STYLE: Identify 'red_offset', 'green_offset', and 'blue_offset' variables manually. 
//    Do NOT use cv::cvtColor to pre-convert the whole image; use the offsets for pixel access.
// 3. Memory & Structure: Initialize the PointCloud2 message.
//    STYLE: Use 'std::make_unique<PointCloud2>()' and 'pcd_modifier.setPointCloud2FieldsByString'.
// 4. Kernel Execution: Trigger the 3D projection.
//    STYLE: You MUST call the 'convertDepth<T>' and 'convertRgb' template functions. 
//    Manual loops are discouraged to maintain consistency with the depth_image_proc library.
//
// DESIGN CONSTRAINTS:
// - Memory Management: Use ROS 2 smart pointer patterns for the output message.
// - Temporal Integrity: The output cloud MUST be perfectly synchronized with the 
//   spatial source (depth_msg) for reliable TF transforms.
// - Field Standards: Use the standard ROS 2 'xyz' and 'rgb' field definitions.
  //END OF TODO
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbNode)

----------------------------


# LLM Response

 // Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.hpp"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depth_image_proc
{

PointCloudXyzrgbNode::PointCloudXyzrgbNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudXyzrgbNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  // value used for invalid points for pcd conversion
  invalid_depth_ = this->declare_parameter<double>("invalid_depth", 0.0);

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync) {
    exact_sync_ = std::make_shared<ExactSynchronizer>(
      ExactSyncPolicy(queue_size),
      sub_depth_,
      sub_rgb_,
      sub_info_);
    exact_sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  } else {
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
    sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  }

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo & s)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (s.current_count == 0) {
        sub_depth_.unsubscribe();
        sub_rgb_.unsubscribe();
        sub_info_.unsubscribe();
      } else if (!sub_depth_.getSubscriber()) {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string depth_topic =
          node_base->resolve_topic_or_service_name("depth_registered/image_rect", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_rect_color", false);
        // Allow also remapping camera_info to something different than default
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
          image_transport::getCameraInfoTopic(rgb_topic), false);

        // parameter for depth_image_transport hint
        image_transport::TransportHints depth_hints(*this,
          "raw", "depth_image_transport");

        rclcpp::SubscriptionOptions sub_opts;
        // Update the subscription options to allow reconfigurable qos settings.
        sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
          {
            // Here all policies that are desired to be reconfigurable are listed.
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability,
          }};

        // depth image can use different transport.(e.g. compressedDepth)
        sub_depth_.subscribe(
          *this, depth_topic,
          depth_hints.getTransport(), rclcpp::SystemDefaultsQoS(), sub_opts);

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints{*this};
        sub_rgb_.subscribe(
          *this,
          rgb_topic,
          hints.getTransport(),
          rclcpp::SystemDefaultsQoS(), sub_opts);
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };
  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SystemDefaultsQoS(),
      pub_options);
}

void PointCloudXyzrgbNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,  // 10 seconds
      "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);
  
  // Handle resolution mismatch: Resize RGB to match Depth if necessary
  Image::ConstSharedPtr rgb_msg = rgb_msg_in;
  
  if (depth_msg->width != rgb_msg_in->width || depth_msg->height != rgb_msg_in->height) {
    // Resize RGB image to match depth resolution
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(rgb_msg_in, rgb_msg_in->encoding);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat rgb_resized;
    cv::resize(cv_ptr->image, rgb_resized, cv::Size(depth_msg->width, depth_msg->height), 
               0, 0, cv::INTER_LINEAR);
    
    cv_bridge::CvImage cv_resized;
    cv_resized.header = rgb_msg_in->header;
    cv_resized.encoding = rgb_msg_in->encoding;
    cv_resized.image = rgb_resized;
    rgb_msg = cv_resized.toImageMsg();
    
    // Scale camera intrinsics manually (fx, fy, cx, cy)
    double scale_x = static_cast<double>(depth_msg->width) / rgb_msg_in->width;
    double scale_y = static_cast<double>(depth_msg->height) / rgb_msg_in->height;
    
    sensor_msgs::msg::CameraInfo scaled_info = *info_msg;
    scaled_info.k[0] *= scale_x;  // fx
    scaled_info.k[2] *= scale_x;  // cx
    scaled_info.k[4] *= scale_y;  // fy
    scaled_info.k[5] *= scale_y;  // cy
    scaled_info.width = depth_msg->width;
    scaled_info.height = depth_msg->height;
    
    model_.fromCameraInfo(scaled_info);
  }
  
  // Determine encoding offsets for RGB8, BGR8, or MONO8
  int red_offset, green_offset, blue_offset, color_step;
  
  if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    red_offset = 0;
    green_offset = 0;
    blue_offset = 0;
    color_step = 1;
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported RGB encoding [%s]", rgb_msg->encoding.c_str());
    return;
  }
  
  // Initialize PointCloud2 message
  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;  // Synchronize with depth for TF integrity
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  
  // Convert depth to XYZ using template function
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepth<uint16_t>(depth_msg, cloud_msg, model_, invalid_depth_);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepth<float>(depth_msg, cloud_msg, model_, invalid_depth_);
  } else {
    RCLCPP_ERROR(get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }
  
  // Convert RGB using offset-based access
  convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  
  // Publish the point cloud
  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbNode)