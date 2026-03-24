/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <angles/angles.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <boost/range/adaptor/reversed.hpp>

namespace cl_move_base_z
{
namespace odom_tracker
{
OdomTracker::OdomTracker(std::string odomTopicName, std::string odomFrame)
  : paramServer_(ros::NodeHandle("~/odom_tracker"))
{
// TODO 1: Node Infrastructure Migration.
    // Initialize thresholds and 'odomFrame_' using ROS 2 parameters. 
    // Set up 'odomSub_' and initialize RealtimePublishers.
    // [STYLE]: 
    // 1. You MUST use 'this->declare_parameter' for all declarations.
    // 2. RealtimePublisher MUST be initialized exactly as: 
    //    std::make_shared<realtime_tools::RealtimePublisher<...>>(this->get_node_base_interface(), ...)
    // 3. Assume this class now inherits from rclcpp::Node.
//END OF TODO
}

/**
 ******************************************************************************************************************
 * setWorkingMode()
 ******************************************************************************************************************
 */
void OdomTracker::setWorkingMode(WorkingMode workingMode)
{
  // ROS_INFO("odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  ROS_INFO("[OdomTracker] setting working mode to: %d", (uint8_t)workingMode);
  workingMode_ = workingMode;
  // ROS_INFO("odom_tracker m_mutex release");
}

/**
 ******************************************************************************************************************
 * setPublishMessages()
 ******************************************************************************************************************
 */
void OdomTracker::setPublishMessages(bool value)
{
  // ROS_INFO("odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  publishMessages = value;
  // ROS_INFO("odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::pushPath(std::string newPathTagName)
{
  ROS_INFO("odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  ROS_INFO("PUSH_PATH PATH EXITING");
  this->logStateString();

  pathStack_.push_back({baseTrajectory_, this->currentPathTagName_});
  baseTrajectory_.poses.clear();

  if(newPathTagName =="")
  {
    this->currentPathTagName_="(unspecified path name)";
  }
  else
  {
    this->currentPathTagName_ = newPathTagName;
  }

  ROS_INFO("PUSH_PATH PATH EXITING");
  this->logStateString();
  ROS_INFO("odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::popPath(int popCount, bool keepPreviousPath)
{
  ROS_INFO("odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  ROS_INFO("POP PATH ENTRY");
  this->logStateString();

  if (!keepPreviousPath)
  {
    baseTrajectory_.poses.clear();
  }

  while (popCount > 0 && !pathStack_.empty())
  {
    auto &stacked = pathStack_.back().path.poses;
    baseTrajectory_.poses.insert(baseTrajectory_.poses.begin(), stacked.begin(), stacked.end());
    pathStack_.pop_back();
    popCount--;

    ROS_INFO("POP PATH Iteration ");
    this->logStateString();
  }

  ROS_INFO("POP PATH EXITING");
  this->logStateString();
  ROS_INFO("odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::logStateString()
{
  ROS_INFO("--- odom tracker state ---");
  ROS_INFO(" - stacked paths count: %ld", pathStack_.size());
  ROS_INFO_STREAM(" - [STACK-HEAD active path '" << currentPathTagName_ <<"' size: "<< baseTrajectory_.poses.size()<<"]");
  int i = 0;
  for (auto &p : pathStack_ | boost::adaptors::reversed)
  {
    ROS_INFO_STREAM(" - p " << i << "[" <<  p.path.header.stamp <<  "][" << p.pathTagName << "], size: " << p.path.poses.size());
    i++;
  }
  ROS_INFO("---");
}

void OdomTracker::clearPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  baseTrajectory_.poses.clear();

  rtPublishPaths(ros::Time::now());
  this->logStateString();
  this->updateAggregatedStackPath();
}

void OdomTracker::setStartPoint(const geometry_msgs::PoseStamped &pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  ROS_INFO_STREAM("[OdomTracker] set current path starting point: " << pose);
  if (baseTrajectory_.poses.size() > 0)
  {
    baseTrajectory_.poses[0] = pose;
  }
  else
  {
    baseTrajectory_.poses.push_back(pose);
  }
  this->updateAggregatedStackPath();
}

void OdomTracker::setStartPoint(const geometry_msgs::Pose &pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  ROS_INFO_STREAM("[OdomTracker] set current path starting point: " << pose);
  geometry_msgs::PoseStamped posestamped;
  posestamped.header.frame_id = this->odomFrame_;
  posestamped.header.stamp = ros::Time::now();
  posestamped.pose = pose;

  if (baseTrajectory_.poses.size() > 0)
  {
    baseTrajectory_.poses[0] = posestamped;
  }
  else
  {
    baseTrajectory_.poses.push_back(posestamped);
  }
  this->updateAggregatedStackPath();
}

nav_msgs::Path OdomTracker::getPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  return this->baseTrajectory_;
}

/**
 ******************************************************************************************************************
 * rtPublishPaths()
 ******************************************************************************************************************
 */
void OdomTracker::rtPublishPaths(ros::Time timestamp)
{
// TODO 2: Implement Realtime-Safe Path Publishing.
    // Use 'trylock()' to ensure non-blocking behavior. 
    // Access message data via '->msg_' and publish using 'unlockAndPublish()'.
    // [STYLE]: Synchronize 'baseTrajectory_' and 'aggregatedStackPathMsg_' 
    // with the provided 'timestamp' before emission.
//END OF TODO
}

void OdomTracker::updateAggregatedStackPath()
{
  aggregatedStackPathMsg_.poses.clear();
  for (auto &p : pathStack_)
  {
    aggregatedStackPathMsg_.poses.insert(aggregatedStackPathMsg_.poses.end(), p.path.poses.begin(), p.path.poses.end());
  }

  aggregatedStackPathMsg_.header.frame_id = this->odomFrame_;
}

/**
 ******************************************************************************************************************
 * updateBackward()
 ******************************************************************************************************************
 */
bool OdomTracker::updateClearPath(const nav_msgs::Odometry &odom)
{
  // we initially accept any message if the queue is empty
  /// Track robot base pose
  geometry_msgs::PoseStamped base_pose;

  base_pose.pose = odom.pose.pose;
  base_pose.header = odom.header;
  baseTrajectory_.header = odom.header;

  bool acceptBackward = false;
  bool clearingError = false;
  bool finished = false;

  while (!finished)
  {
    if (baseTrajectory_.poses.size() <= 1)  // we at least keep always the first point of the forward path when clearing
                                            // (this is important for backwards planner replanning and not losing the
                                            // last goal)
    {
      acceptBackward = false;
      finished = true;
    }
    else
    {
      auto &carrotPose = baseTrajectory_.poses.back().pose;
      const geometry_msgs::Point &carrotPoint = carrotPose.position;
      double carrotAngle = tf::getYaw(carrotPose.orientation);

      auto &currePose = base_pose.pose;
      const geometry_msgs::Point &currePoint = currePose.position;
      double currentAngle = tf::getYaw(currePose.orientation);

      double lastpointdist = p2pDistance(carrotPoint, currePoint);
      double goalAngleOffset = fabs(angles::shortest_angular_distance(carrotAngle, currentAngle));

      acceptBackward = !baseTrajectory_.poses.empty() && lastpointdist < clearPointDistanceThreshold_ &&
                       goalAngleOffset < clearAngularDistanceThreshold_;

      clearingError = lastpointdist > 2 * clearPointDistanceThreshold_;
      ROS_DEBUG_STREAM("[OdomTracker] clearing (accepted: " << acceptBackward << ") linerr: " << lastpointdist
                                                            << ", anglerr: " << goalAngleOffset);
    }

    // ROS_INFO("Backwards, last distance: %lf < %lf accept: %d", dist, minPointDistanceBackwardThresh_,
    // acceptBackward);
    if (acceptBackward && baseTrajectory_.poses.size() > 1) /*we always leave at least one item, specially interesting
                                                               for the backward local planner reach the backwards goal
                                                               with precision enough*/
    {
      baseTrajectory_.poses.pop_back();
    }
    else if (clearingError)
    {
      finished = true;
      ROS_WARN("[OdomTracker] Incorrect odom clearing motion.");
    }
    else
    {
      finished = true;
      /// Not removing point because it is enough far from the last cord point
    }
  }

  return acceptBackward;
}
/**
 ******************************************************************************************************************
 * updateRecordPath()
 ******************************************************************************************************************
 */
bool OdomTracker::updateRecordPath(const nav_msgs::Odometry &odom)
{
  /// Track robot base pose
  geometry_msgs::PoseStamped base_pose;

  base_pose.pose = odom.pose.pose;
  base_pose.header = odom.header;
  baseTrajectory_.header = odom.header;

  bool enqueueOdomMessage = false;

  double dist = -1;
  if (baseTrajectory_.poses.empty())
  {
    enqueueOdomMessage = true;
  }
  else
  {
    const auto &prevPose = baseTrajectory_.poses.back().pose;
    const geometry_msgs::Point &prevPoint = prevPose.position;
    double prevAngle = tf::getYaw(prevPose.orientation);

    const geometry_msgs::Point &currePoint = base_pose.pose.position;
    double currentAngle = tf::getYaw(base_pose.pose.orientation);

    dist = p2pDistance(prevPoint, currePoint);
    double goalAngleOffset = fabs(angles::shortest_angular_distance(prevAngle, currentAngle));

    // ROS_WARN("dist %lf vs min %lf", dist, recordPointDistanceThreshold_);

    if (dist > recordPointDistanceThreshold_ || goalAngleOffset > recordAngularDistanceThreshold_)
    {
      enqueueOdomMessage = true;
    }
    else
    {
      // ROS_WARN("skip odom, dist: %lf", dist);
      enqueueOdomMessage = false;
    }
  }

  if (enqueueOdomMessage)
  {
    baseTrajectory_.poses.push_back(base_pose);
  }

  return enqueueOdomMessage;
}

/**
 ******************************************************************************************************************
 * reconfigCB()
 ******************************************************************************************************************
 */
void OdomTracker::reconfigCB(::move_base_z_client_plugin::OdomTrackerConfig &config, uint32_t level)
{
  ROS_INFO("[OdomTracker] reconfigure Request");
  this->odomFrame_ = config.odom_frame;

  this->recordPointDistanceThreshold_ = config.record_point_distance_threshold;
  this->recordAngularDistanceThreshold_ = config.record_angular_distance_threshold;
  this->clearPointDistanceThreshold_ = config.clear_point_distance_threshold;
  this->clearAngularDistanceThreshold_ = config.clear_angular_distance_threshold;
}

/**
 ******************************************************************************************************************
 * processOdometryMessage()
 ******************************************************************************************************************
 */
void OdomTracker::processOdometryMessage(const nav_msgs::Odometry &odom)
{
  // ROS_INFO("odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  if (workingMode_ == WorkingMode::RECORD_PATH)
  {
    updateRecordPath(odom);
  }
  else if (workingMode_ == WorkingMode::CLEAR_PATH)
  {
    updateClearPath(odom);
  }

  // ROS_WARN("odomTracker odometry callback");
  if (publishMessages)
  {
    rtPublishPaths(odom.header.stamp);
  }

  // ROS_INFO("odom_tracker m_mutex release");
}
}  // namespace odom_tracker
}  // namespace cl_move_base_z
