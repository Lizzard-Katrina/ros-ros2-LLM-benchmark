// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <mutex>

#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  run_without_trigger = false;
  node_->declare_parameter<bool>("run_without_trigger", false);
  node_->get_parameter("run_without_trigger", run_without_trigger);

  if (run_without_trigger)
    RCLCPP_WARN(this->get_logger(), "%s Running without radar trigger", kPrefix.c_str());

  std::string topic_twist = "twist";
  node_->declare_parameter<std::string>("topic_twist", topic_twist);
  node_->get_parameter("topic_twist", topic_twist);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  node_->declare_parameter<std::string>("topic_radar_scan", topic_radar_scan);
  node_->get_parameter("topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/sensor_platform/radar/trigger";
  node_->declare_parameter<std::string>("topic_radar_trigger", topic_radar_trigger);
  node_->get_parameter("topic_radar_trigger", topic_radar_trigger);

  std::string topic_twist_ego_ground_truth = "/ground_truth/twist_radar";
  node_->declare_parameter<std::string>("topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);
  node_->get_parameter("topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);

  sub_radar_scan_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_radar_scan, 50,
      std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarScan, this, std::placeholders::_1));
  sub_radar_trigger_ = node_->create_subscription<std_msgs::msg::Header>(
      topic_radar_trigger, 50,
      std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this, std::placeholders::_1));
  pub_twist_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic_twist, 5);
  pub_twist_ground_truth_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_twist_ego_ground_truth, 5);
}

rclcpp::Logger RadarEgoVelocityEstimatorRos::get_logger() const
{
  return node_->get_logger();
}

void RadarEgoVelocityEstimatorRos::processRadarData(const sensor_msgs::msg::PointCloud2& radar_scan,
                                                    const rclcpp::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp         = trigger_stamp;
    msg.header.frame_id      = (radar_scan.header.frame_id.empty()) ? "radar" : radar_scan.header.frame_id;
    msg.twist.twist.linear.x = v_b_r.x();
    msg.twist.twist.linear.y = v_b_r.y();
    msg.twist.twist.linear.z = v_b_r.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    pub_twist_->publish(msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    RCLCPP_ERROR(this->get_logger(), "%s Radar ego velocity estimation failed", kPrefix.c_str());
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *node_->get_clock(), 5000,
                        "%s Runtime statistics: %s",
                        kPrefix.c_str(),
                        profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::msg::PointCloud2::SharedPtr radar_scan_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (run_without_trigger)
  {
    rclcpp::Time stamp(radar_scan_msg->header.stamp);
    if (radar_scan_msg->header.stamp.sec == 0 && radar_scan_msg->header.stamp.nanosec == 0)
    {
      stamp = this->now();
      RCLCPP_WARN(this->get_logger(), "%s Radar scan timestamp is zero, using this->now()", kPrefix.c_str());
    }
    processRadarData(*radar_scan_msg, stamp);
  }
  else
  {
    if (trigger_stamp.nanoseconds() > 0)
    {
      rclcpp::Time stamp = trigger_stamp;
      if (radar_scan_msg->header.stamp.sec == 0 && radar_scan_msg->header.stamp.nanosec == 0)
      {
        stamp = this->now();
        RCLCPP_WARN(this->get_logger(), "%s Radar scan timestamp is zero, using this->now()", kPrefix.c_str());
      }
      processRadarData(*radar_scan_msg, stamp);
      trigger_stamp = rclcpp::Time();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "%s Radar scan without trigger, skipping", kPrefix.c_str());
    }
  }
}

rclcpp::Time RadarEgoVelocityEstimatorRos::now() const
{
  return node_->get_clock()->now();
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::msg::Header::SharedPtr trigger_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trigger_stamp = rclcpp::Time(trigger_msg->stamp);
}