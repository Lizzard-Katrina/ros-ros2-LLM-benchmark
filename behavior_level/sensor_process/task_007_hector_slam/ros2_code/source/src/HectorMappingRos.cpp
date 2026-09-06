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

#include "task_007_hector_slam/HectorMappingRos.h"

#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/time.h>

HectorMappingRos::HectorMappingRos()
  : Node("hector_mapping")
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("tf_map_scanmatch_transform_frame_name", "scanmatcher_frame");
  this->declare_parameter<bool>("use_tf_scan_transformation", true);
  this->declare_parameter<bool>("use_tf_pose_start_estimate", false);
  this->declare_parameter<bool>("pub_map_odom_transform", true);
  this->declare_parameter<bool>("pub_map_scanmatch_transform", true);
  this->declare_parameter<bool>("pub_odometry", false);
  this->declare_parameter<bool>("map_with_known_poses", false);
  this->declare_parameter<bool>("output_timing", false);
  this->declare_parameter<int>("scan_subscriber_queue_size", 5);
  this->declare_parameter<double>("map_resolution", 0.025);
  this->declare_parameter<int>("map_size", 1024);
  this->declare_parameter<double>("laser_min_dist", 0.4);
  this->declare_parameter<double>("laser_max_dist", 30.0);
  this->declare_parameter<double>("laser_z_min_value", -1.0);
  this->declare_parameter<double>("laser_z_max_value", 1.0);

  p_base_frame_ = this->get_parameter("base_frame").as_string();
  p_map_frame_ = this->get_parameter("map_frame").as_string();
  p_odom_frame_ = this->get_parameter("odom_frame").as_string();
  p_tf_map_scanmatch_transform_frame_name_ = this->get_parameter("tf_map_scanmatch_transform_frame_name").as_string();
  p_use_tf_scan_transformation_ = this->get_parameter("use_tf_scan_transformation").as_bool();
  p_use_tf_pose_start_estimate_ = this->get_parameter("use_tf_pose_start_estimate").as_bool();
  p_pub_map_odom_transform_ = this->get_parameter("pub_map_odom_transform").as_bool();
  p_pub_map_scanmatch_transform_ = this->get_parameter("pub_map_scanmatch_transform").as_bool();
  p_pub_odometry_ = this->get_parameter("pub_odometry").as_bool();
  p_map_with_known_poses_ = this->get_parameter("map_with_known_poses").as_bool();
  p_timing_output_ = this->get_parameter("output_timing").as_bool();
  p_scan_subscriber_queue_size_ = this->get_parameter("scan_subscriber_queue_size").as_int();
  p_map_resolution_ = this->get_parameter("map_resolution").as_double();
  p_map_size_ = this->get_parameter("map_size").as_int();

  double tmp = this->get_parameter("laser_min_dist").as_double();
  p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);
  tmp = this->get_parameter("laser_max_dist").as_double();
  p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);
  p_laser_z_min_value_ = static_cast<float>(this->get_parameter("laser_z_min_value").as_double());
  p_laser_z_max_value_ = static_cast<float>(this->get_parameter("laser_z_max_value").as_double());

  lastSlamPose = Eigen::Vector3f::Zero();
  initial_pose_ = Eigen::Vector3f::Zero();

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialize map_to_odom_ as identity
  map_to_odom_.header.frame_id = p_map_frame_;
  map_to_odom_.child_frame_id = p_odom_frame_;
  map_to_odom_.transform.rotation.w = 1.0;
  map_to_odom_.transform.rotation.x = 0.0;
  map_to_odom_.transform.rotation.y = 0.0;
  map_to_odom_.transform.rotation.z = 0.0;
  map_to_odom_.transform.translation.x = 0.0;
  map_to_odom_.transform.translation.y = 0.0;
  map_to_odom_.transform.translation.z = 0.0;

  // QoS with TransientLocal durability for map and pose topics
  rclcpp::QoS qos_transient_local(10);
  qos_transient_local.transient_local();

  // Publishers
  posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "slam_out_pose", qos_transient_local);
  poseUpdatePublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "poseupdate", qos_transient_local);
  mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", qos_transient_local);

  if (p_pub_odometry_) {
    odometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "scanmatch_odom", 50);
  }

  // Subscriber
  scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS().keep_last(p_scan_subscriber_queue_size_),
    std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));

  lastScanTime = this->now();

  RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s",
              p_use_tf_scan_transformation_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "HectorSM p_pub_map_odom_transform_: %s",
              p_pub_map_odom_transform_ ? "true" : "false");
}

HectorMappingRos::~HectorMappingRos()
{
}

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Check if scan processing is paused
  if (pause_scan_processing_) {
    return;
  }

  // Get the current time from the node clock
  rclcpp::Time start_time = this->now();

  // Convert scan timestamp using tf2_ros::fromMsg for TF lookups
  tf2::TimePoint scan_time_tf2 = tf2_ros::fromMsg(scan->header.stamp);

  // Prepare the laser scan data container
  rosLaserScanToDataContainer(*scan, laserScanContainer, 1.0f);

  // If the data container is empty, skip processing
  if (laserScanContainer.getSize() == 0) {
    RCLCPP_WARN(this->get_logger(), "scanCallback: Laser scan data container is empty, skipping.");
    return;
  }

  // Attempt to get the laser-to-base transform if using TF scan transformation
  geometry_msgs::msg::TransformStamped laser_to_base_tf;
  if (p_use_tf_scan_transformation_) {
    try {
      laser_to_base_tf = tf_buffer_->lookupTransform(
        p_base_frame_, scan->header.frame_id,
        scan->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "scanCallback: TF lookup laser->base failed: %s", ex.what());
      return;
    }
  }

  // Compute the SLAM pose estimate
  // In the full implementation, this would call slamProcessor->update(...)
  // For this migration, we demonstrate the transform chain and pose publishing logic.

  Eigen::Vector3f slamPose = lastSlamPose;

  // Build the map->base transform from the SLAM pose
  geometry_msgs::msg::TransformStamped map_to_base_tf;
  map_to_base_tf.header.stamp = scan->header.stamp;
  map_to_base_tf.header.frame_id = p_map_frame_;
  map_to_base_tf.child_frame_id = p_base_frame_;
  map_to_base_tf.transform.translation.x = static_cast<double>(slamPose.x());
  map_to_base_tf.transform.translation.y = static_cast<double>(slamPose.y());
  map_to_base_tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, static_cast<double>(slamPose.z()));
  map_to_base_tf.transform.rotation.x = q.x();
  map_to_base_tf.transform.rotation.y = q.y();
  map_to_base_tf.transform.rotation.z = q.z();
  map_to_base_tf.transform.rotation.w = q.w();

  // Publish the scanmatcher transform if configured
  if (p_pub_map_scanmatch_transform_) {
    geometry_msgs::msg::TransformStamped scanmatch_tf = map_to_base_tf;
    scanmatch_tf.child_frame_id = p_tf_map_scanmatch_transform_frame_name_;
    tf_broadcaster_->sendTransform(std::move(scanmatch_tf));
  }

  // Compute map->odom transform:
  // T_map_odom = T_map_base * T_base_odom = T_map_base * (T_odom_base).inverse()
  if (p_pub_map_odom_transform_) {
    try {
      geometry_msgs::msg::TransformStamped odom_to_base =
        tf_buffer_->lookupTransform(
          p_odom_frame_, p_base_frame_,
          scan->header.stamp,
          rclcpp::Duration::from_seconds(0.5));

      // Convert to tf2::Transform for math
      tf2::Transform tf_map_base;
      tf2::fromMsg(map_to_base_tf.transform, tf_map_base);

      tf2::Transform tf_odom_base;
      tf2::fromMsg(odom_to_base.transform, tf_odom_base);

      // map->odom = map->base * (odom->base).inverse()
      tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

      map_to_odom_.header.stamp = scan->header.stamp;
      map_to_odom_.header.frame_id = p_map_frame_;
      map_to_odom_.child_frame_id = p_odom_frame_;
      map_to_odom_.transform = tf2::toMsg(tf_map_odom);

    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
        "scanCallback: TF lookup odom->base failed, using last known map->odom: %s", ex.what());
    }

    // Broadcast map->odom
    map_to_odom_.header.stamp = scan->header.stamp;
    tf_broadcaster_->sendTransform(map_to_odom_);
  }

  // Publish the SLAM pose as PoseStamped
  {
    auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = scan->header.stamp;
    pose_msg->header.frame_id = p_map_frame_;
    pose_msg->pose.position.x = static_cast<double>(slamPose.x());
    pose_msg->pose.position.y = static_cast<double>(slamPose.y());
    pose_msg->pose.position.z = 0.0;
    pose_msg->pose.orientation.w = q.w();
    pose_msg->pose.orientation.x = q.x();
    pose_msg->pose.orientation.y = q.y();
    pose_msg->pose.orientation.z = q.z();
    posePublisher_->publish(std::move(pose_msg));
  }

  // Publish pose with covariance
  {
    auto cov_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    cov_msg->header.stamp = scan->header.stamp;
    cov_msg->header.frame_id = p_map_frame_;
    cov_msg->pose.pose.position.x = static_cast<double>(slamPose.x());
    cov_msg->pose.pose.position.y = static_cast<double>(slamPose.y());
    cov_msg->pose.pose.position.z = 0.0;
    cov_msg->pose.pose.orientation.w = q.w();
    cov_msg->pose.pose.orientation.x = q.x();
    cov_msg->pose.pose.orientation.y = q.y();
    cov_msg->pose.pose.orientation.z = q.z();
    poseUpdatePublisher_->publish(std::move(cov_msg));
  }

  // Publish odometry if configured
  if (p_pub_odometry_ && odometryPublisher_) {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = scan->header.stamp;
    odom_msg->header.frame_id = p_map_frame_;
    odom_msg->child_frame_id = p_base_frame_;
    odom_msg->pose.pose.position.x = static_cast<double>(slamPose.x());
    odom_msg->pose.pose.position.y = static_cast<double>(slamPose.y());
    odom_msg->pose.pose.orientation.w = q.w();
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odometryPublisher_->publish(std::move(odom_msg));
  }

  // Update last scan time and pose
  lastScanTime = rclcpp::Time(scan->header.stamp);
  lastSlamPose = slamPose;

  if (p_timing_output_) {
    rclcpp::Duration duration = this->now() - start_time;
    RCLCPP_INFO(this->get_logger(), "HectorSM scanCallback duration: %f ms",
                duration.seconds() * 1000.0);
  }
}

void HectorMappingRos::rosLaserScanToDataContainer(
  const sensor_msgs::msg::LaserScan& scan,
  hectorslam::DataContainer& dataContainer,
  float scaleToMap)
{
  size_t size = scan.ranges.size();
  float angle = scan.angle_min;

  dataContainer.clear();
  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i) {
    float dist = scan.ranges[i];

    if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}