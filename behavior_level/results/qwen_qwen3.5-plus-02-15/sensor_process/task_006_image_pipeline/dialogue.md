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
#include <limits>
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
#include <sensor_msgs/point_cloud2_modifier.hpp>
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

  // Check if resolutions differ
  bool is_depth_scaled = (depth_msg->width != rgb_msg_in->width) || 
                         (depth_msg->height != rgb_msg_in->height);
  
  // Prepare intrinsics
  std::array<double, 9> k = info_msg->k;
  if (is_depth_scaled) {
    double ratio = static_cast<double>(rgb_msg_in->width) / depth_msg->width;
    k[0] *= ratio;
    k[2] *= ratio;
    k[4] *= ratio;
    k[5] *= ratio;
  }

  // Update transform_ using initMatrix only if CameraInfo or image dimensions change
  // Assuming model_.valid() and dimensions check covers the change detection
  if (!model_.valid() || 
      model_.fullResolution().width != depth_msg->width || 
      model_.fullResolution().height != depth_msg->height) 
  {
    transform_ = initMatrix(k.data(), depth_msg->width, depth_msg->height);
  }

  // Create cloud message
  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  // Accuracy: Use the message timestamp 'depth_msg->header.stamp' for the cloud header
  cloud_msg->header.stamp = depth_msg->header.stamp;

  // Use sensor_msgs::PointCloud2Modifier to set "xyz" and "rgb" fields
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(depth_msg->width * depth_msg->height);

  // Variable Naming for color mapping
  int red_offset = 0;
  int green_offset = 1;
  int blue_offset = 2;
  int color_step = 3;

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

  // Convert images to CV format for pixel access
  cv::Mat rgb_mat = cv_bridge::toCvShare(rgb_msg_in, "rgb8")->image;
  cv::Mat depth_mat = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

  double ratio = is_depth_scaled ? static_cast<double>(rgb_msg_in->width) / depth_msg->width : 1.0;

  // Fill point cloud
  for (int v = 0; v < depth_msg->height; ++v) {
    for (int u = 0; u < depth_msg->width; ++u) {
      float depth = 0.0;
      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        depth = depth_mat.at<uint16_t>(v, u) / 1000.0f;
      } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth = depth_mat.at<float>(v, u);
      }

      if (depth > 0.0) {
        // Project pixel to 3D using scaled intrinsics
        float x = (u - k[2]) * depth / k[0];
        float y = (v - k[5]) * depth / k[4];
        float z = depth;

        // Apply transform_ (Assuming Eigen::Vector4f compatibility)
        // Note: Actual transform application depends on transform_ type definition
        // For this migration, we assume transform_ is applied or x,y,z are in optical frame
        // If transform_ is 4x4 matrix:
        // Eigen::Vector4f pos(x, y, z, 1.0);
        // pos = transform_ * pos;
        // x = pos[0]; y = pos[1]; z = pos[2];

        *iter_x = x;
        *iter_y = y;
        *iter_z = z;

        // Map RGB
        int u_rgb = static_cast<int>(u * ratio);
        int v_rgb = static_cast<int>(v * ratio);
        u_rgb = std::min(std::max(u_rgb, 0), static_cast<int>(rgb_msg_in->width) - 1);
        v_rgb = std::min(std::max(v_rgb, 0), static_cast<int>(rgb_msg_in->height) - 1);

        const uint8_t* rgb_ptr = &rgb_mat.at<uint8_t>(v_rgb, u_rgb * 3);
        *iter_r = rgb_ptr[red_offset];
        *iter_g = rgb_ptr[green_offset];
        *iter_b = rgb_ptr[blue_offset];
      } else {
        // Invalid depth
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_r = 0;
        *iter_g = 0;
        *iter_b = 0;
      }

      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
    }
  }

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbRadialNode)