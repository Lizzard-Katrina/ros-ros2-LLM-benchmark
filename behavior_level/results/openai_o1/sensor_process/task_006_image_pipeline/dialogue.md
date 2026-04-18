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
#include <vector>
#include <limits>
#include <array>

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
#include <sensor_msgs/point_cloud2_modifier.hpp>

namespace depth_image_proc
{

static bool transform_initialized_ = false;
static int old_width_ = 0;
static int old_height_ = 0;
static std::array<double, 9> old_k_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

PointCloudXyzrgbRadialNode::PointCloudXyzrgbRadialNode(const rclcpp::NodeOptions & options)
: Node("PointCloudXyzrgbRadialNode", options)
{
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

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
        auto node_base = this->get_node_base_interface();
        std::string depth_topic =
          node_base->resolve_topic_or_service_name("depth/image_raw", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_raw", false);
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
            image_transport::getCameraInfoTopic(rgb_topic), false);

        image_transport::TransportHints depth_hints(*this, "raw", "depth_image_transport");
        sub_depth_.subscribe(*this, depth_topic, depth_hints.getTransport(),
          rclcpp::SystemDefaultsQoS());

        image_transport::TransportHints hints(*this, "raw");
        sub_rgb_.subscribe(*this, rgb_topic, hints.getTransport(), rclcpp::SystemDefaultsQoS());
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };

  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SystemDefaultsQoS(), pub_options);
}

void PointCloudXyzrgbRadialNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN(
      get_logger(), "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  model_.fromCameraInfo(info_msg);

  // BEGIN TODO FILLED
  auto depth_cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  auto rgb_cv_ptr = cv_bridge::toCvShare(rgb_msg_in, rgb_msg_in->encoding);

  if ((depth_msg->width != rgb_msg_in->width) || (depth_msg->height != rgb_msg_in->height)) {
    double ratio_x = static_cast<double>(depth_msg->width) / static_cast<double>(rgb_msg_in->width);
    double ratio_y = static_cast<double>(depth_msg->height) / static_cast<double>(rgb_msg_in->height);
    if (std::fabs(ratio_x - ratio_y) < 1e-6) {
      double ratio = ratio_x;
      auto k = model_.intrinsicMatrix();
      k[0] *= ratio;
      k[2] *= ratio;
      k[4] *= ratio;
      k[5] *= ratio;
      model_.setCameraMatrix(k.data());
    }
  }

  if (!transform_initialized_ ||
      info_msg->width != old_width_ ||
      info_msg->height != old_height_ ||
      std::memcmp(model_.intrinsicMatrix().data(), old_k_.data(), 9 * sizeof(double)) != 0)
  {
    transform_.initMatrix(model_);
    transform_initialized_ = true;
    old_width_ = info_msg->width;
    old_height_ = info_msg->height;
    std::memcpy(old_k_.data(), model_.intrinsicMatrix().data(), 9 * sizeof(double));
  }

  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header.stamp = depth_msg->header.stamp;
  cloud_msg->header.frame_id = depth_msg->header.frame_id;
  cloud_msg->width = depth_msg->width;
  cloud_msg->height = depth_msg->height;
  cloud_msg->is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  modifier.resize(depth_msg->width * depth_msg->height);

  size_t red_offset = modifier.getFieldOffset("rgb");
  size_t green_offset = red_offset + 1;
  size_t blue_offset = red_offset + 2;
  size_t color_step = 4;

  float fx = static_cast<float>(model_.fx());
  float fy = static_cast<float>(model_.fy());
  float cx = static_cast<float>(model_.cx());
  float cy = static_cast<float>(model_.cy());

  uint8_t * data_ptr = reinterpret_cast<uint8_t *>(&cloud_msg->data[0]);

  for (uint32_t v = 0; v < depth_msg->height; ++v) {
    for (uint32_t u = 0; u < depth_msg->width; ++u) {
      float depth_value = 0.0f;
      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        depth_value = static_cast<float>(depth_cv_ptr->image.at<uint16_t>(v, u)) * 0.001f;
      } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_value = depth_cv_ptr->image.at<float>(v, u);
      } else {
        depth_value = std::numeric_limits<float>::quiet_NaN();
      }

      size_t idx = (v * depth_msg->width + u) * modifier.point_step;
      float * px = reinterpret_cast<float *>(data_ptr + idx + modifier.getFieldOffset("x"));
      float * py = reinterpret_cast<float *>(data_ptr + idx + modifier.getFieldOffset("y"));
      float * pz = reinterpret_cast<float *>(data_ptr + idx + modifier.getFieldOffset("z"));

      if (std::isfinite(depth_value) && depth_value > 0.0f) {
        cv::Point3f pt = transform_.getCameraXYZ(u, v, depth_value, fx, fy, cx, cy);
        *px = pt.x;
        *py = pt.y;
        *pz = pt.z;
      } else {
        *px = std::numeric_limits<float>::quiet_NaN();
        *py = std::numeric_limits<float>::quiet_NaN();
        *pz = std::numeric_limits<float>::quiet_NaN();
      }

      uint8_t * pr = data_ptr + idx + red_offset;
      uint8_t * pg = data_ptr + idx + green_offset;
      uint8_t * pb = data_ptr + idx + blue_offset;

      int rgb_v = v;
      int rgb_u = u;
      if (rgb_msg_in->width != depth_msg->width || rgb_msg_in->height != depth_msg->height) {
        double sx = static_cast<double>(rgb_msg_in->width) / static_cast<double>(depth_msg->width);
        double sy = static_cast<double>(rgb_msg_in->height) / static_cast<double>(depth_msg->height);
        rgb_u = static_cast<int>(u * sx);
        rgb_v = static_cast<int>(v * sy);
        if (rgb_u < 0 || rgb_u >= static_cast<int>(rgb_msg_in->width) ||
            rgb_v < 0 || rgb_v >= static_cast<int>(rgb_msg_in->height))
        {
          *pr = 0;
          *pg = 0;
          *pb = 0;
          continue;
        }
      }

      if (rgb_msg_in->encoding == sensor_msgs::image_encodings::RGB8) {
        const auto & c = rgb_cv_ptr->image.at<cv::Vec3b>(rgb_v, rgb_u);
        *pr = c[0];
        *pg = c[1];
        *pb = c[2];
      } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::BGR8) {
        const auto & c = rgb_cv_ptr->image.at<cv::Vec3b>(rgb_v, rgb_u);
        *pr = c[2];
        *pg = c[1];
        *pb = c[0];
      } else {
        *pr = 0;
        *pg = 0;
        *pb = 0;
      }
    }
  }
  // END TODO FILLED

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbRadialNode)
```
