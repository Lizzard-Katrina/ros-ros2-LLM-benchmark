# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: radar_ego_velocity_estimator_ros.cpp
----------------------------
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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(ros::NodeHandle nh)
{
  reconfigure_server_.setCallback(boost::bind(&RadarEgoVelocityEstimatorRos::reconfigureCallback, this, _1, _2));

  run_without_trigger = false;
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "run_without_trigger", run_without_trigger);

  if (run_without_trigger)
    ROS_WARN_STREAM(kPrefix << "Running without radar trigger");

  std::string topic_twist = "twist";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_twist", topic_twist);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/sensor_platform/radar/trigger";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_trigger", topic_radar_trigger);

  std::string topic_twist_ego_ground_truth = "/ground_truth/twist_radar";
  getRosParameter(
      nh, kPrefix, RosParameterType::Recommended, "topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);

  sub_radar_scan_ = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_radar_scan, 50, &RadarEgoVelocityEstimatorRos::callbackRadarScan, this);
  sub_radar_trigger_ = nh.subscribe<std_msgs::Header>(
      topic_radar_trigger, 50, &RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this);
  pub_twist_              = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);
  pub_twist_ground_truth_ = nh.advertise<geometry_msgs::TwistStamped>(topic_twist_ego_ground_truth, 5);
}

void RadarEgoVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
                                                 const double bag_start,
                                                 const double bag_duration,
                                                 const double sleep_ms)
{
  rosbag::Bag source_bag;
  source_bag.open(rosbag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(sub_radar_scan_.getTopic());
  topics.push_back(sub_radar_trigger_.getTopic());
  topics.push_back(pub_twist_ground_truth_.getTopic());

  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  auto first_timestamp = ros::TIME_MIN;

  for (const rosbag::MessageInstance& m : view)
  {
    if (!ros::ok())
      break;

    if (first_timestamp == ros::TIME_MIN)
      first_timestamp = m.getTime();

    if ((m.getTime() - first_timestamp).toSec() < bag_start)
      continue;

    if ((m.getTime() - first_timestamp).toSec() > bag_duration)
      break;

    const auto topic = m.getTopic();
    if (topic == sub_radar_scan_.getTopic())
    {
      const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan != NULL)
      {
        callbackRadarScan(radar_scan);
        if (sleep_ms > 0)
          ros::Duration(sleep_ms / 1.0e3).sleep();
      }
    }
    else if (topic == sub_radar_trigger_.getTopic())
    {
      const auto radar_trigger_msg = m.instantiate<std_msgs::Header>();
      if (radar_trigger_msg != NULL)
        callbackRadarTrigger(radar_trigger_msg);
    }
    else if (topic == pub_twist_ground_truth_.getTopic())
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        pub_twist_ground_truth_.publish(msg);
    }

    ros::spinOnce();
  }

  ROS_INFO("%s Final Runtime statistics: %s",
           kPrefix.c_str(),
           profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::processRadarData(const sensor_msgs::PointCloud2& radar_scan,
                                                    const ros::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::TwistWithCovarianceStamped msg;
    msg.header.stamp         = trigger_stamp;
    msg.header.frame_id      = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
    msg.twist.twist.linear.x = v_b_r.x();
    msg.twist.twist.linear.y = v_b_r.y();
    msg.twist.twist.linear.z = v_b_r.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    pub_twist_.publish(msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    ROS_ERROR_STREAM(kPrefix << "Radar ego velocity estimation failed");
  }

  ROS_INFO_THROTTLE(5,
                    "%s Runtime statistics: %s",
                    kPrefix.c_str(),
                    profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg)
{
/**
 * TODO: Migrate the ROS 1 'callbackRadarScan' logic to ROS 2.
 * * [Interface Requirements]:
 * - Use 'radar_scan_msg' as the input parameter name to match the test suite.
 * - Adhere to the member variables and types defined in the provided ROS 2 header.
 * * [Logic Integrity]:
 * - You must preserve all functional branches from the original ROS 1 code, 
 * including the conditional handling of 'run_without_trigger'.
 * - Ensure that the 'trigger_stamp' state is correctly managed and reset 
 * after the estimation process to prevent stale data usage.
 * - All diagnostic logging (Warnings/Errors) must be migrated to the 
 * equivalent ROS 2 node-based logging system.
 * END OF TODO 
*/
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg)
{
  mutex_.lock();
  trigger_stamp = trigger_msg->stamp;
  mutex_.unlock();
}

----------------------------


# LLM Response



[FILENAME: radar_ego_velocity_estimator_ros.cpp]
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

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(rclcpp::Node::SharedPtr nh)
  : nh_(nh)
{
  run_without_trigger = false;
  getRosParameter(nh_, kPrefix, RosParameterType::Recommended, "run_without_trigger", run_without_trigger);

  if (run_without_trigger)
    RCLCPP_WARN_STREAM(nh_->get_logger(), kPrefix << "Running without radar trigger");

  std::string topic_twist = "twist";
  getRosParameter(nh_, kPrefix, RosParameterType::Recommended, "topic_twist", topic_twist);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  getRosParameter(nh_, kPrefix, RosParameterType::Recommended, "topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/sensor_platform/radar/trigger";
  getRosParameter(nh_, kPrefix, RosParameterType::Recommended, "topic_radar_trigger", topic_radar_trigger);

  std::string topic_twist_ego_ground_truth = "/ground_truth/twist_radar";
  getRosParameter(
      nh_, kPrefix, RosParameterType::Recommended, "topic_twist_radar_ground_truth", topic_twist_ego_ground_truth);

  sub_radar_scan_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_radar_scan, 50, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarScan, this, std::placeholders::_1));
  sub_radar_trigger_ = nh_->create_subscription<std_msgs::msg::Header>(
      topic_radar_trigger, 50, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this, std::placeholders::_1));
  pub_twist_              = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic_twist, 5);
  pub_twist_ground_truth_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_twist_ego_ground_truth, 5);
}

void RadarEgoVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
                                                 const double bag_start,
                                                 const double bag_duration,
                                                 const double sleep_ms)
{
  rosbag2_cpp::Reader reader;
  reader.open(rosbag_path);

  rclcpp::Time first_timestamp(0, 0, RCL_ROS_TIME);
  bool first_timestamp_set = false;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
  rclcpp::Serialization<std_msgs::msg::Header> header_serializer;
  rclcpp::Serialization<geometry_msgs::msg::TwistStamped> twist_serializer;

  while (reader.has_next())
  {
    if (!rclcpp::ok())
      break;

    auto bag_message = reader.read_next();

    rclcpp::Time msg_time(bag_message->time_stamp, RCL_ROS_TIME);

    if (!first_timestamp_set)
    {
      first_timestamp = msg_time;
      first_timestamp_set = true;
    }

    double elapsed = (msg_time - first_timestamp).seconds();

    if (elapsed < bag_start)
      continue;

    if (elapsed > bag_duration)
      break;

    const auto topic = bag_message->topic_name;

    if (topic == sub_radar_scan_->get_topic_name())
    {
      auto radar_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      pc2_serializer.deserialize_message(&serialized_msg, radar_scan.get());
      callbackRadarScan(radar_scan);
      if (sleep_ms > 0)
        rclcpp::sleep_for(std::chrono::microseconds(static_cast<int64_t>(sleep_ms * 1000.0)));
    }
    else if (topic == sub_radar_trigger_->get_topic_name())
    {
      auto radar_trigger_msg = std::make_shared<std_msgs::msg::Header>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      header_serializer.deserialize_message(&serialized_msg, radar_trigger_msg.get());
      callbackRadarTrigger(radar_trigger_msg);
    }
    else if (topic == pub_twist_ground_truth_->get_topic_name())
    {
      auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      twist_serializer.deserialize_message(&serialized_msg, msg.get());
      pub_twist_ground_truth_->publish(*msg);
    }

    rclcpp::spin_some(nh_);
  }

  RCLCPP_INFO(nh_->get_logger(), "%s Final Runtime statistics: %s",
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
    RCLCPP_ERROR_STREAM(nh_->get_logger(), kPrefix << "Radar ego velocity estimation failed");
  }

  RCLCPP_INFO_THROTTLE(nh_->get_logger(),
                    *nh_->get_clock(),
                    5000,
                    "%s Runtime statistics: %s",
                    kPrefix.c_str(),
                    profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::msg::PointCloud2::SharedPtr radar_scan_msg)
{
  if (run_without_trigger)
  {
    processRadarData(*radar_scan_msg, rclcpp::Time(radar_scan_msg->header.stamp));
  }
  else
  {
    mutex_.lock();
    rclcpp::Time local_trigger_stamp = trigger_stamp;
    mutex_.unlock();

    if (local_trigger_stamp.nanoseconds() == 0)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), kPrefix << "No radar trigger received yet, skipping radar scan");
    }
    else
    {
      processRadarData(*radar_scan_msg, local_trigger_stamp);

      mutex_.lock();
      trigger_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      mutex_.unlock();
    }
  }
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::msg::Header::SharedPtr trigger_msg)
{
  mutex_.lock();
  trigger_stamp = rclcpp::Time(trigger_msg->stamp);
  mutex_.unlock();
}