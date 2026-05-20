/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */

#include "slam_gmapping.h"

#include <iostream>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  laser_count_(0), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  map_to_odom_.setRotation(q);
  map_to_odom_.setOrigin(tf2::Vector3(0, 0, 0));
  init();
}

SlamGMapping::SlamGMapping(rclcpp::Node::SharedPtr nh):
  laser_count_(0), node_(nh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  map_to_odom_.setRotation(q);
  map_to_odom_.setOrigin(tf2::Vector3(0, 0, 0));
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  laser_count_(0), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  map_to_odom_.setRotation(q);
  map_to_odom_.setOrigin(tf2::Vector3(0, 0, 0));
  init();
}


void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();
  
  // 1. TF Frames: base_frame, map_frame, odom_frame.
  base_frame_ = node_->declare_parameter("base_frame", "base_link");
  map_frame_ = node_->declare_parameter("map_frame", "map");
  odom_frame_ = node_->declare_parameter("odom_frame", "odom");

  // 2. Scanner Limits: maxRange, maxUrange, minimumScore.
  maxRange_ = node_->declare_parameter("maxRange", 80.0);
  maxUrange_ = node_->declare_parameter("maxUrange", 80.0);
  minimum_score_ = node_->declare_parameter("minimumScore", 0.0);

  if (maxUrange_ > maxRange_) {
    RCLCPP_WARN(node_->get_logger(), "maxUrange cannot be greater than maxRange. Setting maxUrange to maxRange.");
    maxUrange_ = maxRange_;
  }

  // 3. Motion Model (Gaussian Noise): srr, srt, str, stt.
  srr_ = node_->declare_parameter("srr", 0.1);
  srt_ = node_->declare_parameter("srt", 0.2);
  str_ = node_->declare_parameter("str", 0.1);
  stt_ = node_->declare_parameter("stt", 0.2);

  // 4. Grid Resolution: xmin, ymin, xmax, ymax, delta.
  xmin_ = node_->declare_parameter("xmin", -100.0);
  ymin_ = node_->declare_parameter("ymin", -100.0);
  xmax_ = node_->declare_parameter("xmax", 100.0);
  ymax_ = node_->declare_parameter("ymax", 100.0);
  delta_ = node_->declare_parameter("delta", 0.05);

  // 5. Update Strategy: linearUpdate, angularUpdate, temporalUpdate, particles.
  linearUpdate_ = node_->declare_parameter("linearUpdate", 1.0);
  angularUpdate_ = node_->declare_parameter("angularUpdate", 0.5);
  temporalUpdate_ = node_->declare_parameter("temporalUpdate", -1.0);
  particles_ = node_->declare_parameter("particles", 30);
    
  tf_delay_ = node_->declare_parameter("tf_delay", transform_publish_period_);
}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("entropy", 1);
  sst_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  sstm_ = node_->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", 1);
  ss_ = node_->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(node_, "scan", rmw_qos_profile_default);
  scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(*scan_filter_sub_, *tfB_, odom_frame_, 5, node_);
  scan_filter_->registerCallback(std::bind(&SlamGMapping::laserCallback, this, std::placeholders::_1));

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  entropy_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("entropy", 1);
  sst_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  sstm_ = node_->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", 1);
  ss_ = node_->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  rosbag2_cpp::Reader reader;
  reader.open(bag_fname);
  
  // Replay logic simplified for ROS2 migration
  while (reader.has_next()) {
    auto msg = reader.read_next();
    // Process messages...
  }
}

void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  rclcpp::Rate r(1.0 / transform_publish_period);
  while(rclcpp::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
  centered_laser_pose_.stamp_ = t;
  geometry_msgs::msg::TransformStamped odom_pose;
  try
  {
    odom_pose = tfB_->lookupTransform(odom_frame_, centered_laser_pose_.frame_id_, t);
  }
  catch(tf2::TransformException &e)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  
  tf2::Quaternion q(
    odom_pose.transform.rotation.x,
    odom_pose.transform.rotation.y,
    odom_pose.transform.rotation.z,
    odom_pose.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  gmap_pose = GMapping::OrientedPoint(odom_pose.transform.translation.x,
                                      odom_pose.transform.translation.y,
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::msg::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
  geometry_msgs::msg::TransformStamped laser_pose;
  try
  {
    laser_pose = tfB_->lookupTransform(base_frame_, laser_frame_, scan.header.stamp);
  }
  catch(tf2::TransformException &e)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  tf2::Vector3 v(0, 0, 1 + laser_pose.transform.translation.z);
  
  if (fabs(fabs(v.z()) - 1) > 0.001)
  {
    RCLCPP_WARN(node_->get_logger(), "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 v.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (v.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    RCLCPP_INFO(node_->get_logger(), "Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    RCLCPP_INFO(node_->get_logger(), "Laser is mounted upside down.");
  }

  laser_angles_.resize(scan.ranges.size());
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    RCLCPP_WARN(node_->get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  GMapping::sampleGaussian(1,seed_);

  RCLCPP_INFO(node_->get_logger(), "Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::msg::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;
  
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  double* ranges_double = new double[scan.ranges.size()];
  if (do_reverse_range_)
  {
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 rclcpp::Time(scan.header.stamp).seconds());

  delete[] ranges_double;

  reading.setPose(gmap_pose);

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static rclcpp::Time last_map_update(0,0, node_->get_clock()->get_clock_type());

  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(*scan, odom_pose))
  {
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

    tf2::Quaternion q_laser, q_odom;
    q_laser.setRPY(0, 0, mpose.theta);
    q_odom.setRPY(0, 0, odom_pose.theta);
    
    tf2::Transform laser_to_map(q_laser, tf2::Vector3(mpose.x, mpose.y, 0.0));
    laser_to_map = laser_to_map.inverse();
    tf2::Transform odom_to_laser(q_odom, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (rclcpp::Time(scan->header.stamp) - last_map_update).seconds() > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
    }
  }
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::msg::LaserScan& scan)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::msg::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_->publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    if(!n->reading)
    {
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  map_.map.header.stamp = node_->now();
  map_.map.header.frame_id = map_frame_;

  sst_->publish(map_.map);
  sstm_->publish(map_.map.info);
}

bool 
SlamGMapping::mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                          std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res->map = map_.map;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  rclcpp::Time tf_expiration = node_->now() + rclcpp::Duration::from_seconds(tf_delay_);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = tf_expiration;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.translation.x = map_to_odom_.getOrigin().x();
  transform.transform.translation.y = map_to_odom_.getOrigin().y();
  transform.transform.translation.z = map_to_odom_.getOrigin().z();
  transform.transform.rotation.x = map_to_odom_.getRotation().x();
  transform.transform.rotation.y = map_to_odom_.getRotation().y();
  transform.transform.rotation.z = map_to_odom_.getRotation().z();
  transform.transform.rotation.w = map_to_odom_.getRotation().w();
  
  tfB_->sendTransform(transform);
  map_to_odom_mutex_.unlock();
}