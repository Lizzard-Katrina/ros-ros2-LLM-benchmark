# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}


void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();
  //TODO
  // 1. TF Frames: base_frame, map_frame, odom_frame.
  // 2. Scanner Limits: maxRange, maxUrange, minimumScore.
  // 3. Motion Model (Gaussian Noise): srr, srt, str, stt.
  // 4. Grid Resolution: xmin, ymin, xmax, ymax, delta.
  // 5. Update Strategy: linearUpdate, angularUpdate, temporalUpdate, particles.
  //
  // REQUIREMENTS:
  // - Ensure all parameters follow the ROS 2 'Declare-then-Get' pattern.
  // - You must implement internal consistency validation for the laser scanner's 
  //   range parameters to prevent undefined mapping behavior.
  // - Use the ROS 2 logging API for any configuration warnings or status updates.
  // - Map all values to their respective class members (e.g., this->maxUrange_).
  //END OF TODO
    
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback([this](auto msg){ laserCallback(msg); });

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}

void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
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
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
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
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;
  
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  } else
    ROS_DEBUG("cannot process scan");
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
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

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

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

----------------------------


# LLM Response

```cpp
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


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(nullptr), scan_filter_(nullptr), transform_thread_(nullptr)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(rclcpp::Node::SharedPtr& nh, rclcpp::Node::SharedPtr& pnh):
  map_to_odom_(tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(nullptr), scan_filter_(nullptr), transform_thread_(nullptr)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(nullptr), scan_filter_(nullptr), transform_thread_(nullptr),
  seed_(seed), tf_(std::chrono::milliseconds(max_duration_buffer))
{
  init();
}


void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();
  
  // 1. TF Frames: base_frame, map_frame, odom_frame.
  private_nh_->declare_parameter<std::string>("base_frame", "base_link");
  base_frame_ = private_nh_->get_parameter("base_frame").as_string();
  
  private_nh_->declare_parameter<std::string>("map_frame", "map");
  map_frame_ = private_nh_->get_parameter("map_frame").as_string();
  
  private_nh_->declare_parameter<std::string>("odom_frame", "odom");
  odom_frame_ = private_nh_->get_parameter("odom_frame").as_string();
  
  // 2. Scanner Limits: maxRange, maxUrange, minimumScore.
  private_nh_->declare_parameter<double>("maxRange", -1.0);
  maxRange_ = private_nh_->get_parameter("maxRange").as_double();
  
  private_nh_->declare_parameter<double>("maxUrange", -1.0);
  maxUrange_ = private_nh_->get_parameter("maxUrange").as_double();
  
  private_nh_->declare_parameter<double>("minimumScore", 0.0);
  minimum_score_ = private_nh_->get_parameter("minimumScore").as_double();
  
  // Internal consistency validation for laser scanner's range parameters
  if (maxRange_ > 0.0 && maxUrange_ > 0.0) {
    if (maxUrange_ > maxRange_) {
      RCLCPP_WARN(private_nh_->get_logger(),
                  "maxUrange (%.2f) is greater than maxRange (%.2f). Setting maxUrange to maxRange.",
                  maxUrange_, maxRange_);
      maxUrange_ = maxRange_;
      private_nh_->set_parameter(rclcpp::Parameter("maxUrange", maxUrange_));
    }
    if (maxRange_ <= 0.0) {
      RCLCPP_ERROR(private_nh_->get_logger(),
                   "maxRange must be positive, got %.2f. Using default behavior from first scan.",
                   maxRange_);
      maxRange_ = -1.0;
    }
    if (maxUrange_ <= 0.0 && maxUrange_ != -1.0) {
      RCLCPP_ERROR(private_nh_->get_logger(),
                   "maxUrange must be positive or -1 (auto), got %.2f. Using default behavior.",
                   maxUrange_);
      maxUrange_ = -1.0;
    }
  }
  
  // 3. Motion Model (Gaussian Noise): srr, srt, str, stt.
  private_nh_->declare_parameter<double>("srr", 0.1);
  srr_ = private_nh_->get_parameter("srr").as_double();
  
  private_nh_->declare_parameter<double>("srt", 0.2);
  srt_ = private_nh_->get_parameter("srt").as_double();
  
  private_nh_->declare_parameter<double>("str", 0.1);
  str_ = private_nh_->get_parameter("str").as_double();
  
  private_nh_->declare_parameter<double>("stt", 0.2);
  stt_ = private_nh_->get_parameter("stt").as_double();
  
  // 4. Grid Resolution: xmin, ymin, xmax, ymax, delta.
  private_nh_->declare_parameter<double>("xmin", -100.0);
  xmin_ = private_nh_->get_parameter("xmin").as_double();
  
  private_nh_->declare_parameter<double>("ymin", -100.0);
  ymin_ = private_nh_->get_parameter("ymin").as_double();
  
  private_nh_->declare_parameter<double>("xmax", 100.0);
  xmax_ = private_nh_->get_parameter("xmax").as_double();
  
  private_nh_->declare_parameter<double>("ymax", 100.0);
  ymax_ = private_nh_->get_parameter("ymax").as_double();
  
  private_nh_->declare_parameter<double>("delta", 0.05);
  delta_ = private_nh_->get_parameter("delta").as_double();
  
  // 5. Update Strategy: linearUpdate, angularUpdate, temporalUpdate, particles.
  private_nh_->declare_parameter<double>("linearUpdate", 1.0);
  linearUpdate_ = private_nh_->get_parameter("linearUpdate").as_double();
  
  private_nh_->declare_parameter<double>("angularUpdate", 0.5);
  angularUpdate_ = private_nh_->get_parameter("angularUpdate").as_double();
  
  private_nh_->declare_parameter<double>("temporalUpdate", -1.0);
  temporalUpdate_ = private_nh_->get_parameter("temporalUpdate").as_double();
  
  private_nh_->declare_parameter<int>("particles", 30);
  particles_ = private_nh_->get_parameter("particles").as_int();
  
  // Additional GMapping parameters
  private_nh_->declare_parameter<double>("sigma", 0.05);
  sigma_ = private_nh_->get_parameter("sigma").as_double();
  
  private_nh_->declare_parameter<int>("kernelSize", 1);
  kernelSize_ = private_nh_->get_parameter("kernelSize").as_int();
  
  private_nh_->declare_parameter<double>("lstep", 0.05);
  lstep_ = private_nh_->get_parameter("lstep").as_double();
  
  private_nh_->declare_parameter<double>("astep", 0.05);
  astep_ = private_nh_->get_parameter("astep").as_double();
  
  private_nh_->declare_parameter<int>("iterations", 5);
  iterations_ = private_nh_->get_parameter("iterations").as_int();
  
  private_nh_->declare_parameter<double>("lsigma", 0.075);
  lsigma_ = private_nh_->get_parameter("lsigma").as_double();
  
  private_nh_->declare_parameter<double>("ogain", 3.0);
  ogain_ = private_nh_->get_parameter("ogain").as_double();
  
  private_nh_->declare_parameter<int>("lskip", 0);
  lskip_ = private_nh_->get_parameter("lskip").as_int();
  
  private_nh_->declare_parameter<double>("resampleThreshold", 0.5);
  resampleThreshold_ = private_nh_->get_parameter("resampleThreshold").as_double();
  
  private_nh_->declare_parameter<double>("llsamplerange", 0.01);
  llsamplerange_ = private_nh_->get_parameter("llsamplerange").as_double();
  
  private_nh_->declare_parameter<double>("llsamplestep", 0.01);
  llsamplestep_ = private_nh_->get_parameter("llsamplestep").as_double();
  
  private_nh_->declare_parameter<double>("lasamplerange", 0.005);
  lasamplerange_ = private_nh_->get_parameter("lasamplerange").as_double();
  
  private_nh_->declare_parameter<double>("lasamplestep", 0.005);
  lasamplestep_ = private_nh_->get_parameter("lasamplestep").as_double();
  
  private_nh_->declare_parameter<int>("throttle_scans", 1);
  throttle_scans_ = private_nh_->get_parameter("throttle_scans").as_int();
  
  private_nh_->declare_parameter<double>("map_update_interval", 5.0);
  map_update_interval_ = rclcpp::Duration::from_seconds(private_nh_->get_parameter("map_update_interval").as_double());
  
  private_nh_->declare_parameter<double>("transform_publish_period", 0.05);
  transform_publish_period_ = private_nh_->get_parameter("transform_publish_period").as_double();
  
  RCLCPP_INFO(private_nh_->get_logger(), "SlamGMapping parameters loaded successfully.");
    
  private_nh_->declare_parameter<double>("tf_delay", transform_publish_period_);
  tf_delay_ = private_nh_->get_parameter("tf_delay").as_double();
}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::QoS(1).transient_local());
  sst_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
  sstm_ = node_->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::QoS(1).transient_local());
  ss_ = node_->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5, node_);
  scan_filter_->registerCallback([this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg){ laserCallback(msg); });

  transform_thread_ = new std::thread(std::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  rclcpp::Node::SharedPtr private_nh_ = node_->create_sub_node("~");
  entropy_publisher_ = private_nh_->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::QoS(1).transient_local());
  sst_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
  sstm_ = node_->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::QoS(1).transient_local());
  ss_ = node_->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  rosbag2_cpp::StorageOptions storage_options;
  storage_options.uri = bag_fname;
  storage_options.storage_id = "sqlite3";
  
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  std::queue<std::pair<sensor_msgs::msg::LaserScan::ConstSharedPtr, std::string> > s_queue;
  
  while (reader.has_next()) {
    auto serialized_message = reader.read_next();
    
    if (serialized_message->topic_name == "/tf") {
      auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
      rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
      serializer.deserialize_message(&serialized_msg, tf_msg.get());
      
      for (size_t i = 0; i < tf_msg->transforms.size(); ++i) {
        tf_.setTransform(tf_msg->transforms[i], "rosbag");
      }
    }

    if (serialized_message->topic_name == scan_topic) {
      auto s = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
      rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
      serializer.deserialize_message(&serialized_msg, s.get());
      
      if (!(rclcpp::Time(s->header.stamp)).get_clock_type() == RCL_ROS_TIME && s->header.stamp.sec == 0 && s->header.stamp.nanosec == 0)
      {
        s_queue.push(std::make_pair(s, ""));
      }
      if (s_queue.size() > 5) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
    }

    while (!s_queue.empty())
    {
      try
      {
        geometry_msgs::msg::TransformStamped t;
        t = tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, tf2_ros::fromMsg(s_queue.front().first->header.stamp));
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      catch(tf2::TransformException& e)
      {
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
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
  centered_laser_pose_.stamp_ = tf2_ros::fromMsg(t);
  geometry_msgs::msg::TransformStamped odom_pose_stamped;
  try
  {
    odom_pose_stamped = tf_.lookupTransform(odom_frame_, centered_laser_pose_.frame_id_, tf2_ros::fromMsg(t));
  }
  catch(tf2::TransformException e)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf2::getYaw(odom_pose_stamped.transform.rotation);

  gmap_pose = GMapping::OrientedPoint(odom_pose_stamped.transform.translation.x,
                                      odom_pose_stamped.transform.translation.y,
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::msg::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
  geometry_msgs::msg::TransformStamped laser_pose_stamped;
  geometry_msgs::msg::TransformStamped ident_stamped;
  ident_stamped.transform.translation.x = 0;
  ident_stamped.transform.translation.y = 0;
  ident_stamped.transform.translation.z = 0;
  ident_stamped.transform.rotation.w = 1;
  ident_stamped.transform.rotation.x = 0;
  ident_stamped.transform.rotation.y = 0;
  ident_stamped.transform.rotation.z = 0;
  ident_stamped.header.frame_id = laser_frame_;
  ident_stamped.header.stamp = scan.header.stamp;
  try
  {
    laser_pose_stamped = tf_.lookupTransform(base_frame_, laser_frame_, tf2_ros::fromMsg(scan.header.stamp));
  }
  catch(tf2::TransformException e)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  geometry_msgs::msg::Vector3 v;
  v.x = 0;
  v.y = 0;
  v.z = 1 + laser_pose_stamped.transform.translation.z;
  geometry_msgs::msg::TransformStamped up_stamped;
  up_stamped.transform.translation.x = v.x;
  up_stamped.transform.translation.y = v.y;
  up_stamped.transform.translation.z = v.z;
  up_stamped.transform.rotation.w = 1;
  up_stamped.header.frame_id = base_frame_;
  up_stamped.header.stamp = scan.header.stamp;
  try
  {
    geometry_msgs::msg::TransformStamped up_transformed = tf_.lookupTransform(laser_frame_, base_frame_, tf2_ros::fromMsg(scan.header.stamp));
    tf2::Vector3 up_vec(v.x, v.y, v.z);
    tf2::Quaternion q(up_transformed.transform.rotation.x, up_transformed.transform.rotation.y, up_transformed.transform.rotation.z, up_transformed.transform.rotation.w);
    up_vec = tf2::quatRotate(q, up_vec);
    RCLCPP_DEBUG(node_->get_logger(), "Z-Axis in sensor frame: %.3f", up_vec.z());
  }
  catch(tf2::TransformException& e)
  {
    RCLCPP_WARN(node_->get_logger(), "Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  double up_z = up_vec.z();
  if (fabs(fabs(up_z) - 1) > 0.001)
  {
    RCLCPP_WARN(node_->get_logger(), "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up_z);
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (up_z > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(tf2::Transform(q, tf2::Vector3(0,0,0)), tf2_ros::fromMsg(rclcpp::Clock().now()), laser_frame_);
    RCLCPP_INFO(node_->get_logger(), "Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(tf2::Transform(q, tf2::Vector3(0,0,0)), tf2_ros::fromMsg(rclcpp::Clock().now()), laser_frame_);
    RCLCPP_INFO(node_->get_logger(), "Laser is mounted upside down.");
  }

  laser_angles_.resize(scan.ranges.size());
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  RCLCPP_DEBUG(node_->get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  if(maxRange_ < 0.0)
    maxRange_ = scan.range_max - 0.01;
  if(maxUrange_ < 0.0)
    maxUrange_ = maxRange_;

  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  assert(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  assert(gsp_odom_);


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
    RCLCPP_DEBUG(node_->get_logger(), "Inverting scan");
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
                                 rclcppTimeToSec(scan.header.stamp));

  delete[] ranges_double;

  reading.setPose(gmap_pose);

  RCLCPP_DEBUG(node_->get_logger(), "processing scan");

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static rclcpp::Time last_map_update(0, 0, RCL_ROS_TIME);

  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(*scan, odom_pose))
  {
    RCLCPP_DEBUG(node_->get_logger(), "scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    RCLCPP_DEBUG(node_->get_logger(), "new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    RCLCPP_DEBUG(node_->get_logger(), "odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    RCLCPP_DEBUG(node_->get_logger(), "correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf2::Quaternion q1;
    q1.setRPY(0, 0, mpose.theta);
    tf2::Transform laser_to_map = tf2::Transform(q1, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf2::Quaternion q2;
    q2.setRPY(0, 0, odom_pose.theta);
    tf2::Transform odom_to_laser = tf2::Transform(q2, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      RCLCPP_DEBUG(node_->get_logger(), "Updated the map");
    }
  } else
    RCLCPP_DEBUG(node_->get_logger(), "cannot process scan");
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
  RCLCPP_DEBUG(node_->get_logger(), "Update map");
  std::lock_guard<std::mutex> map_lock(map_mutex_);
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

  RCLCPP_DEBUG(node_->get_logger(), "Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    RCLCPP_DEBUG(node_->get_logger(), "  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Reading is NULL");
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
    
    RCLCPP_DEBUG(node_->get_logger(), "map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    RCLCPP_DEBUG(node_->get_logger(), "map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
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
  std::lock_guard<std::mutex> map_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    *res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  rclcpp::Time tf_expiration = node_->now() + rclcpp::Duration::from_seconds(tf_delay_);
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = tf_expiration;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;
  transform_stamped.transform = tf2::tf2ToMsg(map_to_odom_);
  tfB_->sendTransform(transform_stamped);
  map_to_odom_mutex_.unlock();
}
```