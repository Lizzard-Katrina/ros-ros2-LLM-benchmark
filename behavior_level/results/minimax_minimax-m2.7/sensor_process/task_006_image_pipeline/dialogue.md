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

```cpp
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

  // Allocate point cloud message
  std::unique_ptr<PointCloud2> cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header.stamp = depth_msg->header.stamp;
  cloud_msg->header.frame_id = depth_msg->header.frame_id;
  cloud_msg->width = static_cast<uint32_t>(depth_msg->width);
  cloud_msg->height = static_cast<uint32_t>(depth_msg->height);
  cloud_msg->is_dense = false;

  // Set point cloud fields
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Get depth image data
  sensor_msgs::fillImage(*cloud_msg, sensor_msgs::image_encodings::TYPE_32FC1,
                         depth_msg->height, depth_msg->width, depth_msg->step,
                         depth_msg->data);

  // Get camera parameters
  float k0 = info_msg->k[0];
  float k2 = info_msg->k[2];
  float k4 = info_msg->k[4];
  float k5 = info_msg->k[5];

  // Camera scaling if resolutions differ
  float ratio = 1.0f;
  if (static_cast<uint32_t>(depth_msg->width) != info_msg->width ||
      static_cast<uint32_t>(depth_msg->height) != info_msg->height) {
    ratio = static_cast<float>(depth_msg->width) / static_cast<float>(info_msg->width);
    k0 *= ratio;
    k2 *= ratio;
    k4 *= ratio;
    k5 *= ratio;
  }

  // TF/Radial Transform: Update 'transform_' using 'initMatrix' only if CameraInfo or image dimensions change
  if (需要更新transform_) {
    initMatrix(info_msg, depth_msg->width, depth_msg->height);
  }

  // Variable Naming: For color mapping, use exact names: 'red_offset', 'green_offset', 'blue_offset', 'color_step'
  int red_offset = 0;
  int green_offset = 1;
  int blue_offset = 2;
  int color_step = 3;

  // Handle RGB encoding (BGR vs RGB)
  if (rgb_msg_in->encoding == sensor_msgs::image_encodings::BGR8 ||
      rgb_msg_in->encoding == sensor_msgs::image_encodings::BGRA8) {
    red_offset = 2;
    blue_offset = 0;
  }

  // Get RGB data pointer
  const uint8_t* rgb_data = rgb_msg_in->data.data();

  // Compute and publish point cloud
  int row_step = cloud_msg->width * 12;
  cloud_msg->row_step = row_step;
  cloud_msg->data.resize(row_step * cloud_msg->height);
  float* cloud_ptr = reinterpret_cast<float*>(&cloud_msg->data[0]);

  const float* depth_data = reinterpret_cast<const float*>(&depth_msg->data[0]);

  for (size_t v = 0; v < cloud_msg->height; ++v) {
    for (size_t u = 0; u < cloud_msg->width; ++u) {
      float z = depth_data[v * cloud_msg->width + u];

      if (std::isnan(z) || z <= 0) {
        cloud_ptr[0] = cloud_ptr[1] = cloud_ptr[2] = std::numeric_limits<float>::quiet_NaN();
        cloud_ptr[3] = cloud_ptr[4] = cloud_ptr[5] = 0;
      } else {
        float x = (static_cast<float>(u) - k2) / k0 * z;
        float y = (static_cast<float>(v) - k5) / k4 * z;

        cloud_ptr[0] = x;
        cloud_ptr[1] = y;
        cloud_ptr[2] = z;

        int rgb_idx = (v * rgb_msg_in->width + u) * color_step;
        cloud_ptr[3] = static_cast<float>(rgb_data[rgb_idx + red_offset]) / 255.0f;
        cloud_ptr[4] = static_cast<float>(rgb_data[rgb_idx + green_offset]) / 255.0f;
        cloud_ptr[5] = static_cast<float>(rgb_data[rgb_idx + blue_offset]) / 255.0f;
      }
      cloud_ptr += 6;
    }
  }

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbRadialNode)
```