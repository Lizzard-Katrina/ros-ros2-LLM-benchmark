//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.
// BSD License - see original source
//=================================================================================================

#ifndef HECTOR_MAPPING_ROS_H__
#define HECTOR_MAPPING_ROS_H__

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <memory>
#include <cmath>

// Forward declarations for types we don't fully implement
namespace hectorslam {

class DataContainer {
public:
  void clear() { points_.clear(); }
  void setOrigo(const Eigen::Vector2f& origo) { origo_ = origo; }
  void add(const Eigen::Vector2f& point) { points_.push_back(point); }
  size_t getSize() const { return points_.size(); }
  const Eigen::Vector2f& getVecEntry(size_t index) const { return points_[index]; }
  const Eigen::Vector2f& getOrigo() const { return origo_; }
private:
  std::vector<Eigen::Vector2f> points_;
  Eigen::Vector2f origo_ = Eigen::Vector2f::Zero();
};

} // namespace hectorslam


class HectorMappingRos : public rclcpp::Node
{
public:
  HectorMappingRos();
  ~HectorMappingRos();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  void rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan& scan,
                                   hectorslam::DataContainer& dataContainer,
                                   float scaleToMap);

protected:
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseUpdatePublisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

  // Data containers
  hectorslam::DataContainer laserScanContainer;

  // Transform storage
  geometry_msgs::msg::TransformStamped map_to_odom_;

  // Timing
  rclcpp::Time lastScanTime;

  // Pose state
  Eigen::Vector3f lastSlamPose;
  bool initial_pose_set_;
  Eigen::Vector3f initial_pose_;
  bool pause_scan_processing_;

  // Parameters
  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;
  std::string p_tf_map_scanmatch_transform_frame_name_;
  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_pub_map_odom_transform_;
  bool p_pub_map_scanmatch_transform_;
  bool p_pub_odometry_;
  bool p_map_with_known_poses_;
  bool p_timing_output_;
  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;
  double p_map_resolution_;
  int p_map_size_;
  int p_scan_subscriber_queue_size_;
};

#endif