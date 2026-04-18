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
#include <depth_image_proc/point_cloud_xyzrgb_radial.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace depth_image_proc
{

PointCloudXyzrgbRadialNode::PointCloudXyzrgbRadialNode(const rclcpp::NodeOptions & options)
: Node("PointCloudXyzrgbRadialNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync) {
    exact_sync_ = std::make_unique<ExactSynchronizer>(
      ExactSyncPolicy(queue_size),
      sub_depth_,
      sub_rgb_,
      sub_info_);
    exact_sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbRadialNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  } else {
    sync_ =
      std::make_unique<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
    sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbRadialNode::imageCb,
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
          node_base->resolve_topic_or_service_name("depth/image_raw", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_raw", false);
        // Allow also remapping camera_info to something different than default
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
          image_transport::getCameraInfoTopic(rgb_topic), false);

        // depth image can use different transport.(e.g. compressedDepth)
        image_transport::TransportHints depth_hints(*this, "raw", "depth_image_transport");
        sub_depth_.subscribe(*this, depth_topic, depth_hints.getTransport(),
          rclcpp::SystemDefaultsQoS());

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints(*this, "raw");
        sub_rgb_.subscribe(*this, rgb_topic, hints.getTransport(), rclcpp::SystemDefaultsQoS());
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };
  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SystemDefaultsQoS(),
      pub_options);
}

void PointCloudXyzrgbRadialNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN(
      get_logger(), "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);
/**
   * TODO: Migrate the depth-to-RGB pointcloud conversion logic to ROS 2.
   * [Style & Logic Constraints]:
   * 1. Camera Scaling: If resolutions differ, you MUST scale intrinsics explicitly 
   * using 'k[0] *= ratio', 'k[2] *= ratio', 'k[4] *= ratio', and 'k[5] *= ratio'. 
   * Do NOT use a loop for all 9 elements to avoid scaling the constant '1.0' at k[8].
   * 2. Variable Naming: For color mapping, you MUST use the exact variable names: 
   * 'red_offset', 'green_offset', 'blue_offset', and 'color_step'.
   * 3. TF/Radial Transform: Update 'transform_' using 'initMatrix' only if CameraInfo or 
   * image dimensions change.
   * 4. Memory: Use 'std::unique_ptr<PointCloud2>' for 'cloud_msg' and 'std::move' in publish().
   * 5. PointCloud2: Use 'sensor_msgs::PointCloud2Modifier' to set "xyz" and "rgb" fields.
   * 6. Accuracy: Use the message timestamp 'depth_msg->header.stamp' for the cloud header.
   *END OF TODO   
*/
  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbRadialNode)

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
#include <vector>

#include "cv_bridge/cv_bridge.hpp"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb_radial.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace depth_image_proc
{

PointCloudXyzrgbRadialNode::PointCloudXyzrgbRadialNode(const rclcpp::NodeOptions & options)
: Node("PointCloudXyzrgbRadialNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync) {
    exact_sync_ = std::make_unique<ExactSynchronizer>(
      ExactSyncPolicy(queue_size),
      sub_depth_,
      sub_rgb_,
      sub_info_);
    exact_sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbRadialNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  } else {
    sync_ =
      std::make_unique<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
    sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbRadialNode::imageCb,
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
          node_base->resolve_topic_or_service_name("depth/image_raw", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_raw", false);
        // Allow also remapping camera_info to something different than default
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
          image_transport::getCameraInfoTopic(rgb_topic), false);

        // depth image can use different transport.(e.g. compressedDepth)
        image_transport::TransportHints depth_hints(*this, "raw", "depth_image_transport");
        sub_depth_.subscribe(*this, depth_topic, depth_hints.getTransport(),
          rclcpp::SystemDefaultsQoS());

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints(*this, "raw");
        sub_rgb_.subscribe(*this, rgb_topic, hints.getTransport(), rclcpp::SystemDefaultsQoS());
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };
  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SystemDefaultsQoS(),
      pub_options);
}

void PointCloudXyzrgbRadialNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN(
      get_logger(), "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare(rgb_msg_in, rgb_msg_in->encoding);
  cv::Mat rgb_image = cv_ptr_rgb->image;
  Image::ConstSharedPtr rgb_msg = rgb_msg_in;

  sensor_msgs::msg::CameraInfo scaled_info = *info_msg;
  if (depth_msg->width != rgb_msg_in->width || depth_msg->height != rgb_msg_in->height) {
    const double ratio_x = static_cast<double>(depth_msg->width) /
      static_cast<double>(rgb_msg_in->width);
    const double ratio_y = static_cast<double>(depth_msg->height) /
      static_cast<double>(rgb_msg_in->height);

    cv::resize(
      cv_ptr_rgb->image, rgb_image, cv::Size(depth_msg->width, depth_msg->height), 0.0, 0.0,
      cv::INTER_LINEAR);

    scaled_info.width = depth_msg->width;
    scaled_info.height = depth_msg->height;
    scaled_info.k[0] *= ratio_x;
    scaled_info.k[2] *= ratio_x;
    scaled_info.k[4] *= ratio_y;
    scaled_info.k[5] *= ratio_y;
    scaled_info.p[0] *= ratio_x;
    scaled_info.p[2] *= ratio_x;
    scaled_info.p[5] *= ratio_y;
    scaled_info.p[6] *= ratio_y;
    scaled_info.binning_x = 0;
    scaled_info.binning_y = 0;
    scaled_info.roi.x_offset = 0;
    scaled_info.roi.y_offset = 0;
    scaled_info.roi.height = 0;
    scaled_info.roi.width = 0;
    scaled_info.roi.do_rectify = false;

    auto rgb_resized = std::make_shared<Image>(*rgb_msg_in);
    rgb_resized->width = depth_msg->width;
    rgb_resized->height = depth_msg->height;
    rgb_resized->step = static_cast<sensor_msgs::msg::Image::_step_type>(rgb_image.step);
    rgb_msg = rgb_resized;
  }

  if (
    !transform_ ||
    radial_info_.width != scaled_info.width ||
    radial_info_.height != scaled_info.height ||
    radial_info_.k != scaled_info.k ||
    radial_info_.d != scaled_info.d)
  {
    radial_info_ = scaled_info;
    transform_ = initMatrix(radial_info_);
  }

  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->header.stamp = depth_msg->header.stamp;
  cloud_msg->header.frame_id = depth_msg->header.frame_id;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_bigendian = false;
  cloud_msg->is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(depth_msg->width * depth_msg->height);

  int red_offset = 0;
  int green_offset = 1;
  int blue_offset = 2;
  int color_step = 3;

  if (rgb_msg_in->encoding == sensor_msgs::image_encodings::RGB8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::BGR8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::RGBA8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 4;
  } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::BGRA8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 4;
  } else {
    cv::Mat converted;
    if (cv_ptr_rgb->image.channels() == 4) {
      cv::cvtColor(rgb_image, converted, cv::COLOR_BGRA2BGR);
    } else if (cv_ptr_rgb->image.channels() == 3) {
      converted = rgb_image;
    } else {
      cv::cvtColor(rgb_image, converted, cv::COLOR_GRAY2BGR);
    }
    rgb_image = converted;
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  }

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepthRadial<uint16_t>(
      depth_msg,
      *cloud_msg,
      *transform_,
      model_.cx(),
      model_.cy(),
      model_.fx(),
      model_.fy());
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepthRadial<float>(
      depth_msg,
      *cloud_msg,
      *transform_,
      model_.cx(),
      model_.cy(),
      model_.fx(),
      model_.fy());
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

  for (uint32_t v = 0; v < depth_msg->height; ++v) {
    const uint8_t * rgb_row = rgb_image.ptr<uint8_t>(v);
    for (uint32_t u = 0; u < depth_msg->width; ++u, ++iter_r, ++iter_g, ++iter_b) {
      const uint8_t * rgb_pixel = rgb_row + (u * color_step);
      *iter_r = rgb_pixel[red_offset];
      *iter_g = rgb_pixel[green_offset];
      *iter_b = rgb_pixel[blue_offset];
    }
  }

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbRadialNode)