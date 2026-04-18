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
        image_transport::TransportHints depth_hints(this, "raw", "depth_image_transport");
        sub_depth_.subscribe(this, depth_topic, depth_hints.getTransport(),
          rclcpp::SystemDefaultsQoS());

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints(this, "raw");
        sub_rgb_.subscribe(this, rgb_topic, hints.getTransport(), rclcpp::SystemDefaultsQoS());
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

  // TODO: Migrate the depth-to-RGB pointcloud conversion logic to ROS 2.
  // [Style & Logic Constraints]:
  // 1. Camera Scaling: If resolutions differ, you MUST scale intrinsics explicitly 
  // using 'k[0] *= ratio', 'k[2] *= ratio', 'k[4] *= ratio', and 'k[5] *= ratio'. 
  // Do NOT use a loop for all 9 elements to avoid scaling the constant '1.0' at k[8].
  // 2. Variable Naming: For color mapping, you MUST use the exact variable names: 
  // 'red_offset', 'green_offset', 'blue_offset', and 'color_step'.
  // 3. TF/Radial Transform: Update 'transform_' using 'initMatrix' only if CameraInfo or 
  // image dimensions change.
  // 4. Memory: Use 'std::unique_ptr<PointCloud2>' for 'cloud_msg' and 'std::move' in publish().
  // 5. PointCloud2: Use 'sensor_msgs::PointCloud2Modifier' to set "xyz" and "rgb" fields.
  // 6. Accuracy: Use the message timestamp 'depth_msg->header.stamp' for the cloud header.
  //END OF TODO

  // Check if we need to update the transform matrix
  if (info_msg->width != transform_.width || info_msg->height != transform_.height) {
    transform_.initMatrix(model_);
  }

  // Convert depth image
  cv::Mat depth;
  try {
    depth = cv_bridge::toCvShare(depth_msg)->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert RGB image
  cv::Mat rgb;
  try {
    rgb = cv_bridge::toCvShare(rgb_msg_in, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Handle resolution differences
  double ratio = static_cast<double>(rgb.cols) / static_cast<double>(depth.cols);
  if (ratio != 1.0) {
    // Scale camera intrinsics
    auto& k = model_.intrinsicMatrix();
    k(0,0) *= ratio;  // fx
    k(0,2) *= ratio;  // cx
    k(1,1) *= ratio;  // fy
    k(1,2) *= ratio;  // cy
  }

  // Create point cloud message
  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth.rows;
  cloud_msg->width = depth.cols;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  // Set up point cloud fields
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

  // Set up iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

  // Define color mapping variables
  const int red_offset = 0;
  const int green_offset = 1;
  const int blue_offset = 2;
  const int color_step = 3;

  // Convert depth image to point cloud with color
  for (int v = 0; v < depth.rows; ++v) {
    for (int u = 0; u < depth.cols; ++u) {
      float depth_val = depth.at<float>(v, u);
      
      if (std::isfinite(depth_val) && depth_val > 0) {
        // Convert to 3D point
        cv::Point3f pt = transform_.projectPixelTo3dRay(cv::Point2f(u, v)) * depth_val;
        
        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;

        // Map color from RGB image
        int rgb_u = static_cast<int>(u * ratio);
        int rgb_v = static_cast<int>(v * ratio);
        
        if (rgb_u >= 0 && rgb_u < rgb.cols && rgb_v >= 0 && rgb_v < rgb.rows) {
          cv::Vec3b color = rgb.at<cv::Vec3b>(rgb_v, rgb_u);
          *iter_r = color[2];  // Red
          *iter_g = color[1];  // Green
          *iter_b = color[0];  // Blue
        } else {
          *iter_r = 0;
          *iter_g = 0;
          *iter_b = 0;
        }
      } else {
        // Invalid point
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_r = *iter_g = *iter_b = 0;
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
```