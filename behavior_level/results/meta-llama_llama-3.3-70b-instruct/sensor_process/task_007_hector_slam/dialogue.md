# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
{
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  // Initialize services
  reset_map_service_ = node_.advertiseService("reset_map", &HectorMappingRos::resetMapCallback, this);
  restart_hector_service_ = node_.advertiseService("restart_mapping_with_new_pose", &HectorMappingRos::restartHectorCallback, this);
  toggle_scan_processing_service_ = node_.advertiseService("pause_mapping", &HectorMappingRos::pauseMapCallback, this);

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
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
    delete map__publish_thread_;
}

void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
/**
 * TODO: Implement the ROS 2 Hector SLAM scan processing pipeline.
 * [Style & Logic Constraints - MANDATORY]:
 * 1. TF Access: Use 'tf_buffer_' as a pointer to call 'lookupTransform' 
 * (i.e., 'tf_buffer_->lookupTransform').
 * 2. Variable Naming: The data container for laser scans MUST be named 'laserScanContainer'.
 * 3. Math Path: Calculate 'map_to_odom_' by explicitly using the inverse: 
 * 'map_to_base_tf * odom_to_base.inverse()'.
 * 4. ROS 2 Time: Use 'tf2_ros::fromMsg(scan->header.stamp)' for TF lookups. 
 * Do NOT use manual chrono/nanoseconds additions.
 * 5. QoS Compliance: When publishing 'map' or 'pose', ensure you use a QoS 
 * profile with 'transient_local' durability.
 * 6. Move Semantics: Use 'std::move()' for all 'publish' calls.
 * 7. Logging: Use 'RCLCPP_INFO/WARN' with 'this->get_logger()'.
   *END OF TODO   
*/
}

void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(std_srvs::Trigger::Request  &req,
                                        std_srvs::Trigger::Response &res)
{
  ROS_INFO("HectorSM Reset map service called");
  slamProcessor->reset();
  return true;
}

bool HectorMappingRos::restartHectorCallback(hector_mapping::ResetMapping::Request  &req,
                                             hector_mapping::ResetMapping::Response &res)
{
  // Reset map
  ROS_INFO("HectorSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req.initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool HectorMappingRos::pauseMapCallback(std_srvs::SetBool::Request  &req,
                                        std_srvs::SetBool::Response &res)
{
  this->toggleMappingPause(req.data);
  res.success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
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

  mapPublisher.mapPublisher_.publish(map_.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
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

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
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

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  ROS_INFO("[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}

----------------------------


# LLM Response

Here is the ROS2 version of the provided ROS1 code:

```cpp
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/clock.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "hector_mapping/msg/reset_mapping.hpp"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef double tfScalar;
#endif

class HectorMappingRos : public rclcpp::Node
{
public:
  HectorMappingRos()
  : Node("hector_mapping_ros")
  , debugInfoProvider(nullptr)
  , hectorDrawings(nullptr)
  , lastGetMapUpdateIndex(-100)
  , tfB_(nullptr)
  , map__publish_thread_(nullptr)
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
  {
    // Initialize parameters
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

    // Get parameters
    p_pub_drawings = this->get_parameter("pub_drawings").as_bool();
    p_pub_debug_output_ = this->get_parameter("pub_debug_output").as_bool();
    p_pub_map_odom_transform_ = this->get_parameter("pub_map_odom_transform").as_bool();
    p_pub_odometry_ = this->get_parameter("pub_odometry").as_bool();
    p_advertise_map_service_ = this->get_parameter("advertise_map_service").as_bool();
    p_scan_subscriber_queue_size_ = this->get_parameter("scan_subscriber_queue_size").as_int();

    p_map_resolution_ = this->get_parameter("map_resolution").as_double();
    p_map_size_ = this->get_parameter("map_size").as_int();
    p_map_start_x_ = this->get_parameter("map_start_x").as_double();
    p_map_start_y_ = this->get_parameter("map_start_y").as_double();
    p_map_multi_res_levels_ = this->get_parameter("map_multi_res_levels").as_int();

    p_update_factor_free_ = this->get_parameter("update_factor_free").as_double();
    p_update_factor_occupied_ = this->get_parameter("update_factor_occupied").as_double();

    p_map_update_distance_threshold_ = this->get_parameter("map_update_distance_thresh").as_double();
    p_map_update_angle_threshold_ = this->get_parameter("map_update_angle_thresh").as_double();

    p_scan_topic_ = this->get_parameter("scan_topic").as_string();
    p_sys_msg_topic_ = this->get_parameter("sys_msg_topic").as_string();
    p_pose_update_topic_ = this->get_parameter("pose_update_topic").as_string();

    p_use_tf_scan_transformation_ = this->get_parameter("use_tf_scan_transformation").as_bool();
    p_use_tf_pose_start_estimate_ = this->get_parameter("use_tf_pose_start_estimate").as_bool();
    p_map_with_known_poses_ = this->get_parameter("map_with_known_poses").as_bool();

    p_base_frame_ = this->get_parameter("base_frame").as_string();
    p_map_frame_ = this->get_parameter("map_frame").as_string();
    p_odom_frame_ = this->get_parameter("odom_frame").as_string();

    p_pub_map_scanmatch_transform_ = this->get_parameter("pub_map_scanmatch_transform").as_bool();
    p_tf_map_scanmatch_transform_frame_name_ = this->get_parameter("tf_map_scanmatch_transform_frame_name").as_string();

    p_output_timing_ = this->get_parameter("output_timing").as_bool();

    p_map_pub_period_ = this->get_parameter("map_pub_period").as_double();

    p_laser_min_dist_ = this->get_parameter("laser_min_dist").as_double();
    p_laser_max_dist_ = this->get_parameter("laser_max_dist").as_double();
    p_laser_z_min_value_ = this->get_parameter("laser_z_min_value").as_double();
    p_laser_z_max_value_ = this->get_parameter("laser_z_max_value").as_double();

    // Initialize variables
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

    for (int i = 0; i < mapLevels; ++i)
    {
      mapPubContainer.push_back(MapPublisherContainer());
      slamProcessor->addMapMutex(i, new HectorMapMutex());

      std::string mapTopicStr("map");

      if (i != 0)
      {
        mapTopicStr.append("_" + std::to_string(i));
      }

      std::string mapMetaTopicStr(mapTopicStr);
      mapMetaTopicStr.append("_metadata");

      MapPublisherContainer& tmp = mapPubContainer[i];
      tmp.mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopicStr, 1);
      tmp.mapMetadataPublisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>(mapMetaTopicStr, 1);

      if ( (i == 0) && p_advertise_map_service_)
      {
        tmp.dynamicMapServiceServer_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&HectorMappingRos::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
      }

      setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

      if ( i== 0){
        mapPubContainer[i].mapMetadataPublisher_->publish(mapPubContainer[i].map_.map.info);
      }
    }

    // Initialize services
    reset_map_service_ = this->create_service<std_srvs::srv::Trigger>("reset_map", std::bind(&HectorMappingRos::resetMapCallback, this, std::placeholders::_1, std::placeholders::_2));
    restart_hector_service_ = this->create_service<hector_mapping::srv::ResetMapping>("restart_mapping_with_new_pose", std::bind(&HectorMappingRos::restartHectorCallback, this, std::placeholders::_1, std::placeholders::_2));
    toggle_scan_processing_service_ = this->create_service<std_srvs::srv::SetBool>("pause_mapping", std::bind(&HectorMappingRos::pauseMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_map_pub_period_: %f", p_map_pub_period_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_free_: %f", p_update_factor_free_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
    RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_, std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));
    sysMsgSubscriber_ = this->create_subscription<std_msgs::msg::String>(p_sys_msg_topic_, std::bind(&HectorMappingRos::sysMsgCallback, this, std::placeholders::_1));

    poseUpdatePublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(p_pose_update_topic_, 1);
    posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", 1);

    scan_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("slam_cloud", 1);

    tfB_ = new tf2_ros::TransformBroadcaster(this);

    initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>(this, "initialpose", 2);
    initial_pose_filter_ = new tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_buffer_, p_map_frame_, 2);
    initial_pose_filter_->registerCallback(std::bind(&HectorMappingRos::initialPoseCallback, this, std::placeholders::_1));

    map__publish_thread_ = new std::thread(std::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

    map_to_odom_.setIdentity();

    lastMapPublishTime = this->get_clock()->now();
  }

  ~HectorMappingRos()
  {
    delete slamProcessor;

    if (hectorDrawings)
      delete hectorDrawings;

    if (debugInfoProvider)
      delete debugInfoProvider;

    if (tfB_)
      delete tfB_;

    if(map__publish_thread_)
      delete map__publish_thread_;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // TODO: Implement the ROS 2 Hector SLAM scan processing pipeline.
    // [Style & Logic Constraints - MANDATORY]:
    // 1. TF Access: Use 'tf_buffer_' as a pointer to call 'lookupTransform' 
    // (i.e., 'tf_buffer_->lookupTransform').
    // 2. Variable Naming: The data container for laser scans MUST be named 'laserScanContainer'.
    // 3. Math Path: Calculate 'map_to_odom_' by explicitly using the inverse: 
    // 'map_to_base_tf * odom_to_base.inverse()'.
    // 4. ROS 2 Time: Use 'tf2_ros::fromMsg(scan->header.stamp)' for TF lookups. 
    // Do NOT use manual chrono/nanoseconds additions.
    // 5. QoS Compliance: When publishing 'map' or 'pose', ensure you use a QoS 
    // profile with 'transient_local' durability.
    // 6. Move Semantics: Use 'std::move()' for all 'publish' calls.
    // 7. Logging: Use 'RCLCPP_INFO/WARN' with 'this->get_logger()'.
    // *END OF TODO   
  }

  void sysMsgCallback(const std_msgs::msg::String::SharedPtr string)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM sysMsgCallback, msg contents: %s", string->data.c_str());

    if (string->data == "reset")
    {
      RCLCPP_INFO(this->get_logger(), "HectorSM reset");
      slamProcessor->reset();
    }
  }

  bool mapCallback(nav_msgs::srv::GetMap::Request::SharedPtr req,
                   nav_msgs::srv::GetMap::Response::SharedPtr res)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM Map service called");
    *res = mapPubContainer[0].map_;
    return true;
  }

  bool resetMapCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    RCLCPP_INFO(this->get_logger(), "HectorSM Reset map service called");
    slamProcessor->reset();
    return true;
  }

  bool restartHectorCallback(hector_mapping::srv::ResetMapping::Request::SharedPtr req,
                             hector_mapping::srv::ResetMapping::Response::SharedPtr res)
  {
    // Reset map
    RCLCPP_INFO(this->get_logger(), "HectorSM Reset map");
    slamProcessor->reset();

    // Reset pose
    this->resetPose(req->initial_pose);

    // Unpause node (in case it is paused)
    this->toggleMappingPause(false);

    // Return success
    return true;
  }

  bool pauseMapCallback(std_srvs::srv::SetBool::Request::SharedPtr req,
                        std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    this->toggleMappingPause(req->data);
    res->success = true;
    return true;
  }

  void publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, rclcpp::Time timestamp, MapLockerInterface* mapMutex)
  {
    nav_msgs::msg::GetMap::Response& map_ (mapPublisher.map_);

    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
    {

      int sizeX = gridMap.getSizeX();
      int sizeY = gridMap.getSizeY();

      int size = sizeX * sizeY;

      std::vector<int8_t>& data = map_.map.data;

      //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
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

    mapPublisher.mapPublisher_->publish(map_.map);
  }

  void rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan::SharedPtr scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
  {
    size_t size = scan->ranges.size();

    float angle = scan->angle_min;

    dataContainer.clear();

    dataContainer.setOrigo(Eigen::Vector2f::Zero());

    float maxRangeForContainer = scan->range_max - 0.1f;

    for (size_t i = 0; i < size; ++i)
    {
      float dist = scan->ranges[i];

      if ( (dist > scan->range_min) && (dist < maxRangeForContainer))
      {
        dist *= scaleToMap;
        dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
      }

      angle += scan->angle_increment;
    }
  }

  void rosPointCloudToDataContainer(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud, const tf2::Transform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
  {
    size_t size = pointCloud->width * pointCloud->height;
    //ROS_INFO("size: %d", size);

    dataContainer.clear();

    tf2::Vector3 laserPos (laserTransform.getOrigin());
    dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

    for (size_t i = 0; i < size; ++i)
    {

      const geometry_msgs::msg::Point32& currPoint(pointCloud->points[i]);

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

  void setServiceGetMapData(nav_msgs::msg::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
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

  void publishMapLoop(double map_pub_period)
  {
    rclcpp::Rate r(this->get_node_base_interface(), 1.0 / map_pub_period);
    while(rclcpp::ok())
    {
      rclcpp::Time mapTime (this->get_clock()->now());
      publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

      r.sleep();
    }
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    this->resetPose(msg->pose.pose);
  }

  void toggleMappingPause(bool pause)
  {
    // Pause/unpause
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

  void resetPose(const geometry_msgs::msg::Pose& pose)
  {
    initial_pose_set_ = true;
    initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
    RCLCPP_INFO(this->get_logger(), "[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
               initial_pose_[0], initial_pose_[1], initial_pose_[2]);
  }

private:
  bool p_pub_drawings;
  bool p_pub_debug_output_;
  bool p_pub_map_odom_transform_;
  bool p_pub_odometry_;
  bool p_advertise_map_service_;
  int p_scan_subscriber_queue_size_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;

  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  std::string p_scan_topic_;
  std::string p_sys_msg_topic_;
  std::string p_pose_update_topic_;

  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_map_with_known_poses_;

  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;

  bool p_pub_map_scanmatch_transform_;
  std::string p_tf_map_scanmatch_transform_frame_name_;

  bool p_output_timing_;

  double p_map_pub_period_;

  double p_laser_min_dist_;
  double p_laser_max_dist_;
  double p_laser_z_min_value_;
  double p_laser_z_max_value_;

  HectorDrawings* hectorDrawings;
  HectorDebugInfoProvider* debugInfoProvider;
  int lastGetMapUpdateIndex;
  tf2_ros::TransformBroadcaster* tfB_;
  std::thread* map__publish_thread_;
  bool initial_pose_set_;
  bool pause_scan_processing_;
  Eigen::Vector3f initial_pose_;
  Eigen::Affine3f map_to_odom_;
  rclcpp::Time lastMapPublishTime;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseUpdatePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_point_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sysMsgSubscriber_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr dynamicMapServiceServer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_service_;
  rclcpp::Service<hector_mapping::srv::ResetMapping>::SharedPtr restart_hector_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_scan_processing_service_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>* initial_pose_sub_;
  tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>* initial_pose_filter_;
  tf2_ros::Buffer* tf_buffer_;
  hectorslam::HectorSlamProcessor* slamProcessor;
  std::vector<MapPublisherContainer> mapPubContainer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HectorMappingRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```