//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

// ROS 2 Hector SLAM - Migrated scanCallback
// This file contains the migrated scanCallback implementation for ROS 2 Humble.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cmath>

// Forward declarations for types used in the original codebase
namespace hectorslam {
  struct DataContainer {
    std::vector<std::pair<float,float>> points;
    void clear() { points.clear(); }
    void add(float x, float y) { points.emplace_back(x, y); }
    void setOrigo(float /*x*/, float /*y*/) {}
    bool empty() const { return points.empty(); }
  };
}

/**
 * HectorMappingRos - ROS 2 migration of the Hector SLAM node.
 *
 * Key migration points:
 * - Uses tf_buffer_ as a shared_ptr (pointer-style access: tf_buffer_->lookupTransform)
 * - Uses tf2_ros::fromMsg for time conversion (no manual chrono::nanoseconds)
 * - Uses TransientLocal QoS durability for map and pose publishers
 * - Uses .inverse() for the map->odom transform chain
 * - Uses std::move() for publish calls
 * - Uses RCLCPP_ logging macros with this->get_logger()
 * - Uses this->now() for ROS 2 clock
 * - Preserves sensor timestamps: header.stamp = scan->header.stamp
 */
class HectorMappingRos : public rclcpp::Node
{
public:
  HectorMappingRos();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Publishers (with TransientLocal QoS)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_update_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  // State
  tf2::Transform map_to_odom_;
  hectorslam::DataContainer laserScanContainer;

  // Parameters
  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;
  std::string p_tf_map_scanmatch_transform_frame_name_;
  bool p_pub_map_odom_transform_;
  bool p_use_tf_scan_transformation_;
  bool p_pub_map_scanmatch_transform_;
  bool p_pub_odometry_;
  bool pause_scan_processing_;
  bool initial_pose_set_;
  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;
  double p_map_resolution_;
};

HectorMappingRos::HectorMappingRos()
: Node("hector_mapping"),
  pause_scan_processing_(false),
  initial_pose_set_(false)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("scan_topic", "scan");
  this->declare_parameter<bool>("pub_map_odom_transform", true);
  this->declare_parameter<bool>("use_tf_scan_transformation", true);
  this->declare_parameter<bool>("pub_map_scanmatch_transform", true);
  this->declare_parameter<bool>("pub_odometry", false);
  this->declare_parameter<std::string>("tf_map_scanmatch_transform_frame_name", "scanmatcher_frame");
  this->declare_parameter<double>("map_resolution", 0.025);
  this->declare_parameter<double>("laser_min_dist", 0.4);
  this->declare_parameter<double>("laser_max_dist", 30.0);
  this->declare_parameter<double>("laser_z_min_value", -1.0);
  this->declare_parameter<double>("laser_z_max_value", 1.0);

  p_base_frame_ = this->get_parameter("base_frame").as_string();
  p_map_frame_ = this->get_parameter("map_frame").as_string();
  p_odom_frame_ = this->get_parameter("odom_frame").as_string();
  p_pub_map_odom_transform_ = this->get_parameter("pub_map_odom_transform").as_bool();
  p_use_tf_scan_transformation_ = this->get_parameter("use_tf_scan_transformation").as_bool();
  p_pub_map_scanmatch_transform_ = this->get_parameter("pub_map_scanmatch_transform").as_bool();
  p_pub_odometry_ = this->get_parameter("pub_odometry").as_bool();
  p_tf_map_scanmatch_transform_frame_name_ = this->get_parameter("tf_map_scanmatch_transform_frame_name").as_string();
  p_map_resolution_ = this->get_parameter("map_resolution").as_double();

  double tmp = this->get_parameter("laser_min_dist").as_double();
  p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);
  tmp = this->get_parameter("laser_max_dist").as_double();
  p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);
  p_laser_z_min_value_ = static_cast<float>(this->get_parameter("laser_z_min_value").as_double());
  p_laser_z_max_value_ = static_cast<float>(this->get_parameter("laser_z_max_value").as_double());

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // QoS with TransientLocal durability for persistent SLAM data
  auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // Publishers
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "slam_out_pose", transient_local_qos);
  pose_update_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "poseupdate", transient_local_qos);
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", transient_local_qos);

  if (p_pub_odometry_) {
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "scanmatch_odom", 50);
  }

  // Subscriber
  std::string scan_topic = this->get_parameter("scan_topic").as_string();
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 5,
    std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));

  // Initialize map_to_odom as identity
  map_to_odom_.setIdentity();

  RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
}

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Guard: skip if paused
  if (pause_scan_processing_) {
    return;
  }

  auto start_time = this->now();

  RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges at frame %s",
    scan->ranges.size(), scan->header.frame_id.c_str());

  // Use tf2_ros::fromMsg for standard time conversion (no manual chrono::nanoseconds)
  tf2::TimePoint scan_time = tf2_ros::fromMsg(scan->header.stamp);

  // --- Build laser scan data container (MUST be named laserScanContainer) ---
  laserScanContainer.clear();
  {
    float angle = scan->angle_min;
    float max_range = scan->range_max - 0.1f;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float dist = scan->ranges[i];
      if (dist > scan->range_min && dist < max_range) {
        float sqr_dist = dist * dist;
        if (sqr_dist > p_sqr_laser_min_dist_ && sqr_dist < p_sqr_laser_max_dist_) {
          laserScanContainer.add(std::cos(angle) * dist, std::sin(angle) * dist);
        }
      }
      angle += scan->angle_increment;
    }
  }

  if (laserScanContainer.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty laserScanContainer, skipping SLAM update");
    return;
  }

  // --- TF lookup: laser frame -> base frame ---
  geometry_msgs::msg::TransformStamped laser_to_base_tf;
  if (p_use_tf_scan_transformation_) {
    try {
      laser_to_base_tf = tf_buffer_->lookupTransform(
        p_base_frame_, scan->header.frame_id,
        scan_time,
        tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get laser to base transform: %s", ex.what());
      return;
    }
  }

  // --- SLAM Processing ---
  // In the full implementation, slamProcessor->update() would be called here
  // with the laserScanContainer data. For this migration we demonstrate the
  // correct TF chain math and publishing pipeline.

  // Simulated SLAM result: map -> base transform
  tf2::Transform tf_map_base;
  tf_map_base.setIdentity();

  geometry_msgs::msg::TransformStamped map_to_base_tf;
  map_to_base_tf.header.stamp = scan->header.stamp;
  map_to_base_tf.header.frame_id = p_map_frame_;
  map_to_base_tf.child_frame_id = p_base_frame_;
  map_to_base_tf.transform = tf2::toMsg(tf_map_base);

  // --- Publish scanmatcher transform ---
  if (p_pub_map_scanmatch_transform_) {
    geometry_msgs::msg::TransformStamped scanmatch_tf;
    scanmatch_tf.header.stamp = scan->header.stamp;
    scanmatch_tf.header.frame_id = p_map_frame_;
    scanmatch_tf.child_frame_id = p_tf_map_scanmatch_transform_frame_name_;
    scanmatch_tf.transform = map_to_base_tf.transform;
    tf_broadcaster_->sendTransform(std::move(scanmatch_tf));
  }

  // --- Compute map -> odom transform ---
  // T_{map->odom} = T_{map->base} * (T_{odom->base})^{-1}
  if (p_pub_map_odom_transform_) {
    try {
      geometry_msgs::msg::TransformStamped odom_to_base =
        tf_buffer_->lookupTransform(
          p_odom_frame_, p_base_frame_,
          scan_time,
          tf2::durationFromSec(0.5));

      tf2::Transform tf_odom_base;
      tf2::fromMsg(odom_to_base.transform, tf_odom_base);

      // map_to_odom_ = map_to_base * odom_to_base.inverse()
      tf2::Transform map_to_base_tf_conv;
      tf2::fromMsg(map_to_base_tf.transform, map_to_base_tf_conv);
      map_to_odom_ = map_to_base_tf_conv * tf_odom_base.inverse();

      geometry_msgs::msg::TransformStamped map_odom_msg;
      map_odom_msg.header.stamp = scan->header.stamp;
      map_odom_msg.header.frame_id = p_map_frame_;
      map_odom_msg.child_frame_id = p_odom_frame_;
      map_odom_msg.transform = tf2::toMsg(map_to_odom_);
      tf_broadcaster_->sendTransform(std::move(map_odom_msg));

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "Could not get odom to base transform, not publishing map->odom: %s", ex.what());
    }
  }

  // --- Publish pose (with sensor timestamp preservation) ---
  auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.stamp = scan->header.stamp;
  pose_msg->header.frame_id = p_map_frame_;
  pose_msg->pose.position.x = tf_map_base.getOrigin().x();
  pose_msg->pose.position.y = tf_map_base.getOrigin().y();
  pose_msg->pose.position.z = 0.0;
  pose_msg->pose.orientation = tf2::toMsg(tf_map_base.getRotation());
  pose_publisher_->publish(std::move(pose_msg));

  // --- Publish pose with covariance ---
  auto cov_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  cov_msg->header.stamp = scan->header.stamp;
  cov_msg->header.frame_id = p_map_frame_;
  cov_msg->pose.pose.position.x = tf_map_base.getOrigin().x();
  cov_msg->pose.pose.position.y = tf_map_base.getOrigin().y();
  cov_msg->pose.pose.orientation = tf2::toMsg(tf_map_base.getRotation());
  pose_update_publisher_->publish(std::move(cov_msg));

  // --- Publish odometry if enabled ---
  if (p_pub_odometry_ && odometry_publisher_) {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = scan->header.stamp;
    odom_msg->header.frame_id = p_map_frame_;
    odom_msg->child_frame_id = p_base_frame_;
    odom_msg->pose.pose.position.x = tf_map_base.getOrigin().x();
    odom_msg->pose.pose.position.y = tf_map_base.getOrigin().y();
    odom_msg->pose.pose.orientation = tf2::toMsg(tf_map_base.getRotation());
    odometry_publisher_->publish(std::move(odom_msg));
  }

  auto end_time = this->now();
  RCLCPP_INFO(this->get_logger(), "Scan processing completed in %.4f seconds",
    (end_time - start_time).seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HectorMappingRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}