//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

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

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef tf2Scalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
  : rclcpp::Node("hector_mapping")
  , debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
{
  std::string mapTopic_ = "map";

  this->declare_parameter("pub_drawings", false);
  this->declare_parameter("pub_debug_output", false);
  this->declare_parameter("pub_map_odom_transform", true);
  this->declare_parameter("pub_odometry", false);
  this->declare_parameter("advertise_map_service", true);
  this->declare_parameter("scan_subscriber_queue_size", 5);

  this->declare_parameter("map_resolution", 0.025);
  this->declare_parameter("map_size", 1024);
  this->declare_parameter("map_start_x", 0.5);
  this->declare_parameter("map_start_y", 0.5);
  this->declare_parameter("map_multi_res_levels", 3);

  this->declare_parameter("update_factor_free", 0.4);
  this->declare_parameter("update_factor_occupied", 0.9);

  this->declare_parameter("map_update_distance_thresh", 0.4);
  this->declare_parameter("map_update_angle_thresh", 0.9);

  this->declare_parameter("scan_topic", std::string("scan"));
  this->declare_parameter("sys_msg_topic", std::string("syscommand"));
  this->declare_parameter("pose_update_topic", std::string("poseupdate"));

  this->declare_parameter("use_tf_scan_transformation", true);
  this->declare_parameter("use_tf_pose_start_estimate", false);
  this->declare_parameter("map_with_known_poses", false);

  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("odom_frame", std::string("odom"));

  this->declare_parameter("pub_map_scanmatch_transform", true);
  this->declare_parameter("tf_map_scanmatch_transform_frame_name", std::string("scanmatcher_frame"));

  this->declare_parameter("output_timing", false);
  this->declare_parameter("map_pub_period", 2.0);

  this->declare_parameter("laser_min_dist", 0.4);
  this->declare_parameter("laser_max_dist", 30.0);
  this->declare_parameter("laser_z_min_value", -1.0);
  this->declare_parameter("laser_z_max_value", 1.0);

  this->get_parameter("pub_drawings", p_pub_drawings);
  this->get_parameter("pub_debug_output", p_pub_debug_output_);
  this->get_parameter("pub_map_odom_transform", p_pub_map_odom_transform_);
  this->get_parameter("pub_odometry", p_pub_odometry_);
  this->get_parameter("advertise_map_service", p_advertise_map_service_);
  this->get_parameter("scan_subscriber_queue_size", p_scan_subscriber_queue_size_);

  this->get_parameter("map_resolution", p_map_resolution_);
  this->get_parameter("map_size", p_map_size_);
  this->get_parameter("map_start_x", p_map_start_x_);
  this->get_parameter("map_start_y", p_map_start_y_);
  this->get_parameter("map_multi_res_levels", p_map_multi_res_levels_);

  this->get_parameter("update_factor_free", p_update_factor_free_);
  this->get_parameter("update_factor_occupied", p_update_factor_occupied_);

  this->get_parameter("map_update_distance_thresh", p_map_update_distance_threshold_);
  this->get_parameter("map_update_angle_thresh", p_map_update_angle_threshold_);

  this->get_parameter("scan_topic", p_scan_topic_);
  this->get_parameter("sys_msg_topic", p_sys_msg_topic_);
  this->get_parameter("pose_update_topic", p_pose_update_topic_);

  this->get_parameter("use_tf_scan_transformation", p_use_tf_scan_transformation_);
  this->get_parameter("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_);
  this->get_parameter("map_with_known_poses", p_map_with_known_poses_);

  this->get_parameter("base_frame", p_base_frame_);
  this->get_parameter("map_frame", p_map_frame_);
  this->get_parameter("odom_frame", p_odom_frame_);

  this->get_parameter("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_);
  this->get_parameter("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_);

  this->get_parameter("output_timing", p_timing_output_);
  this->get_parameter("map_pub_period", p_map_pub_period_);

  double tmp = 0.0;
  this->get_parameter("laser_min_dist", tmp);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  this->get_parameter("laser_max_dist", tmp);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  this->get_parameter("laser_z_min_value", tmp);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  this->get_parameter("laser_z_max_value", tmp);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + std::to_string(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp_container = mapPubContainer[i];
    tmp_container.mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopicStr, map_qos);
    tmp_container.mapMetadataPublisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>(mapMetaTopicStr, map_qos);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp_container.dynamicMapServiceServer_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&HectorMappingRos::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    setServiceGetMapData(tmp_container.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      tmp_container.mapMetadataPublisher_->publish(mapPubContainer[i].map_.map.info);
    }
  }

  reset_map_service_ = this->create_service<std_srvs::srv::Trigger>("reset_map", std::bind(&HectorMappingRos::resetMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  restart_hector_service_ = this->create_service<hector_mapping::srv::ResetMapping>("restart_mapping_with_new_pose", std::bind(&HectorMappingRos::restartHectorCallback, this, std::placeholders::_1, std::placeholders::_2));
  toggle_scan_processing_service_ = this->create_service<std_srvs::srv::SetBool>("pause_mapping", std::bind(&HectorMappingRos::pauseMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_, p_scan_subscriber_queue_size_, std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));
  sysMsgSubscriber_ = this->create_subscription<std_msgs::msg::String>(p_sys_msg_topic_, 2, std::bind(&HectorMappingRos::sysMsgCallback, this, std::placeholders::_1));

  poseUpdatePublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(p_pose_update_topic_, 1);
  posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", map_qos);

  scan_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("slam_cloud", 1);

  tfB_ = new tf2_ros::TransformBroadcaster(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>(this, "initialpose", rmw_qos_profile_default);
  initial_pose_filter_ = new tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_buffer_, p_map_frame_, 2, this->get_node_logging_interface(), this->get_node_clock_interface());
  initial_pose_filter_->registerCallback(std::bind(&HectorMappingRos::initialPoseCallback, this, std::placeholders::_1));

  map__publish_thread_ = new std::thread(std::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
  {
    map__publish_thread_->join();
    delete map__publish_thread_;
  }
}

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (pause_scan_processing_)
  {
    return;
  }

  rclcpp::Time scan_time = tf2_ros::fromMsg(scan->header.stamp);

  try
  {
    geometry_msgs::msg::TransformStamped laser_transform = tf_buffer_->lookupTransform(
      p_base_frame_, scan->header.frame_id, scan_time, rclcpp::Duration::from_seconds(0.5));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  hectorslam::DataContainer laserScanContainer;
  rosLaserScanToDataContainer(*scan, laserScanContainer, slamProcessor->getScaleToMap());

  slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());

  tf2::Transform map_to_base_tf;
  Eigen::Vector3f matchPose = slamProcessor->getLastScanMatchPose();
  map_to_base_tf.setOrigin(tf2::Vector3(matchPose.x(), matchPose.y(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, matchPose.z());
  map_to_base_tf.setRotation(q);

  tf2::Transform odom_to_base;
  try
  {
    geometry_msgs::msg::TransformStamped odom_transform = tf_buffer_->lookupTransform(
      p_odom_frame_, p_base_frame_, scan_time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(odom_transform.transform, odom_to_base);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  map_to_odom_ = map_to_base_tf * odom_to_base.inverse();

  if (p_pub_map_odom_transform_)
  {
    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = scan_time;
    map_to_odom_msg.header.frame_id = p_map_frame_;
    map_to_odom_msg.child_frame_id = p_odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom_);
    tfB_->sendTransform(map_to_odom_msg);
  }

  auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.stamp = scan_time;
  pose_msg->header.frame_id = p_map_frame_;
  pose_msg->pose.position.x = matchPose.x();
  pose_msg->pose.position.y = matchPose.y();
  pose_msg->pose.position.z = 0.0;
  pose_msg->pose.orientation = tf2::toMsg(q);

  posePublisher_->publish(std::move(pose_msg));
}

void HectorMappingRos::sysMsgCallback(const std_msgs::msg::String::SharedPtr string)
{
  RCLCPP_INFO(this->get_logger(), "HectorSM sysMsgCallback, msg contents: %s", string->data.c_str());

  if (string->data == "reset")
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  (void)req;
  RCLCPP_INFO(this->get_logger(), "HectorSM Map service called");
  *res = mapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  RCLCPP_INFO(this->get_logger(), "HectorSM Reset map service called");
  slamProcessor->reset();
  res->success = true;
  return true;
}

bool HectorMappingRos::restartHectorCallback(const std::shared_ptr<hector_mapping::srv::ResetMapping::Request> req,
                                             std::shared_ptr<hector_mapping::srv::ResetMapping::Response> res)
{
  (void)res;
  RCLCPP_INFO(this->get_logger(), "HectorSM Reset map");
  slamProcessor->reset();

  this->resetPose(req->initial_pose);

  this->toggleMappingPause(false);

  return true;
}

bool HectorMappingRos::pauseMapCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  this->toggleMappingPause(req->data);
  res->success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, rclcpp::Time timestamp, HectorMapMutex* mapMutex)
{
  nav_msgs::srv::GetMap::Response& map_ (mapPublisher.map_);

  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_.map);
  mapPublisher.mapPublisher_->publish(std::move(map_msg));
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::msg::PointCloud& pointCloud, const tf2::Transform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();

  dataContainer.clear();

  tf2::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::msg::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf2::Vector3 pointPosBaseFrame(laserTransform * tf2::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::srv::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()/1000*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  rclcpp::Rate r(1.0 / map_pub_period);
  while(rclcpp::ok())
  {
    rclcpp::Time mapTime (this->get_clock()->now());
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  if (pause && !pause_scan_processing_)
  {
    RCLCPP_INFO(this->get_logger(), "[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    RCLCPP_INFO(this->get_logger(), "[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::msg::Pose &pose)
{
  initial_pose_set_ = true;
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, yaw);
  RCLCPP_INFO(this->get_logger(), "[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}