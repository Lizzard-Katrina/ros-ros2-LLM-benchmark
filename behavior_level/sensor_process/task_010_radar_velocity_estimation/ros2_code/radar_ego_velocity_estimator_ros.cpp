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

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(rclcpp::Node::SharedPtr nh)
: node_(nh)
{
  run_without_trigger = false;
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "run_without_trigger", run_without_trigger);

  if (run_without_trigger)
    RCLCPP_WARN_STREAM(node_->get_logger(), kPrefix << "Running without radar trigger");

  std::string topic_twist = "twist";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_twist", topic_twist);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/sensor_platform/radar/trigger";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_trigger", topic_radar_trigger);

  std::string topic_twist_ego_ground_truth = "/ground_truth/twist_radar";
  getRosParameter(
      nh, kPrefix, RosParameterType::Recommended, "topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);

  sub_radar_scan_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_radar_scan, 50, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarScan, this, std::placeholders::_1));
  sub_radar_trigger_ = nh->create_subscription<std_msgs::msg::Header>(
      topic_radar_trigger, 50, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this, std::placeholders::_1));
  pub_twist_              = nh->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic_twist, 5);
  pub_twist_ground_truth_ = nh->create_publisher<geometry_msgs::msg::TwistStamped>(topic_twist_ego_ground_truth, 5);
}

void RadarEgoVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
                                                 const double bag_start,
                                                 const double bag_duration,
                                                 const double sleep_ms)
{
  rosbag2_cpp::Reader reader;
  reader.open(rosbag_path);

  auto first_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> scan_serialization;
  rclcpp::Serialization<std_msgs::msg::Header> trigger_serialization;
  rclcpp::Serialization<geometry_msgs::msg::TwistStamped> gt_serialization;

  while (reader.has_next() && rclcpp::ok())
  {
    auto m = reader.read_next();
    rclcpp::Time msg_time(m->time_stamp);

    if (first_timestamp.nanoseconds() == 0)
      first_timestamp = msg_time;

    if ((msg_time - first_timestamp).seconds() < bag_start)
      continue;

    if ((msg_time - first_timestamp).seconds() > bag_duration)
      break;

    const auto topic = m->topic_name;
    rclcpp::SerializedMessage serialized_msg(*m->serialized_data);

    if (topic == sub_radar_scan_->get_topic_name())
    {
      auto radar_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
      scan_serialization.deserialize_message(&serialized_msg, radar_scan.get());
      callbackRadarScan(radar_scan);
      if (sleep_ms > 0)
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_ms)));
    }
    else if (topic == sub_radar_trigger_->get_topic_name())
    {
      auto radar_trigger_msg = std::make_shared<std_msgs::msg::Header>();
      trigger_serialization.deserialize_message(&serialized_msg, radar_trigger_msg.get());
      callbackRadarTrigger(radar_trigger_msg);
    }
    else if (topic == pub_twist_ground_truth_->get_topic_name())
    {
      auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
      gt_serialization.deserialize_message(&serialized_msg, msg.get());
      pub_twist_ground_truth_->publish(*msg);
    }

    rclcpp::spin_some(node_);
  }

  RCLCPP_INFO(node_->get_logger(), "%s Final Runtime statistics: %s",
           kPrefix.c_str(),
           profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
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
    msg.header.frame_id      = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
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
    RCLCPP_ERROR_STREAM(node_->get_logger(), kPrefix << "Radar ego velocity estimation failed");
  }

  auto& clk = *node_->get_clock();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), clk, 5000,
                    "%s Runtime statistics: %s",
                    kPrefix.c_str(),
                    profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr radar_scan_msg)
{
  mutex_.lock();
  rclcpp::Time current_trigger_stamp = trigger_stamp;
  mutex_.unlock();

  if (run_without_trigger)
  {
    processRadarData(*radar_scan_msg, radar_scan_msg->header.stamp);
  }
  else
  {
    if (current_trigger_stamp.nanoseconds() == 0)
    {
      RCLCPP_WARN(node_->get_logger(), "%s Received radar scan but no trigger stamp available", kPrefix.c_str());
    }
    else
    {
      processRadarData(*radar_scan_msg, current_trigger_stamp);
      mutex_.lock();
      trigger_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      mutex_.unlock();
    }
  }
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::msg::Header::ConstSharedPtr trigger_msg)
{
  mutex_.lock();
  trigger_stamp = trigger_msg->stamp;
  mutex_.unlock();
}