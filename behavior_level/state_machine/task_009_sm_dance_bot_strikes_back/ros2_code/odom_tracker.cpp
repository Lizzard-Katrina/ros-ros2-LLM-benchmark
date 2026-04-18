/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <angles/angles.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <boost/range/adaptor/reversed.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_move_base_z
{
namespace odom_tracker
{
OdomTracker::OdomTracker(std::string odomTopicName, std::string odomFrame)
: rclcpp::Node("odom_tracker")
{
  this->declare_parameter<std::string>("odom_frame", odomFrame);
  this->declare_parameter<double>("record_point_distance_threshold", 0.10);
  this->declare_parameter<double>("record_angular_distance_threshold", 0.10);
  this->declare_parameter<double>("clear_point_distance_threshold", 0.10);
  this->declare_parameter<double>("clear_angular_distance_threshold", 0.10);
  this->declare_parameter<bool>("publish_messages", true);
  this->declare_parameter<std::string>("robot_base_path_topic", "odom_tracker_path");
  this->declare_parameter<std::string>("robot_base_path_stacked_topic", "odom_tracker_stacked_path");

  this->get_parameter("odom_frame", this->odomFrame_);
  this->get_parameter("record_point_distance_threshold", this->recordPointDistanceThreshold_);
  this->get_parameter("record_angular_distance_threshold", this->recordAngularDistanceThreshold_);
  this->get_parameter("clear_point_distance_threshold", this->clearPointDistanceThreshold_);
  this->get_parameter("clear_angular_distance_threshold", this->clearAngularDistanceThreshold_);
  this->get_parameter("publish_messages", this->publishMessages);

  std::string robotBasePathTopic;
  std::string robotBasePathStackedTopic;
  this->get_parameter("robot_base_path_topic", robotBasePathTopic);
  this->get_parameter("robot_base_path_stacked_topic", robotBasePathStackedTopic);

  baseTrajectory_.header.frame_id = this->odomFrame_;
  aggregatedStackPathMsg_.header.frame_id = this->odomFrame_;

  odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odomTopicName, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->processOdometryMessage(*msg); });

  robotBasePathPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>>(
    this->get_node_base_interface(), robotBasePathTopic, 1);

  robotBasePathStackedPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>>(
    this->get_node_base_interface(), robotBasePathStackedTopic, 1);
}

/**
 ******************************************************************************************************************
 * setWorkingMode()
 ******************************************************************************************************************
 */
void OdomTracker::setWorkingMode(WorkingMode workingMode)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO(this->get_logger(), "[OdomTracker] setting working mode to: %d", (uint8_t)workingMode);
  workingMode_ = workingMode;
}

/**
 ******************************************************************************************************************
 * setPublishMessages()
 ******************************************************************************************************************
 */
void OdomTracker::setPublishMessages(bool value)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  publishMessages = value;
  this->updateAggregatedStackPath();
}

void OdomTracker::pushPath(std::string newPathTagName)
{
  RCLCPP_INFO(this->get_logger(), "odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO(this->get_logger(), "PUSH_PATH PATH EXITING");
  this->logStateString();

  pathStack_.push_back({baseTrajectory_, this->currentPathTagName_});
  baseTrajectory_.poses.clear();

  if (newPathTagName == "")
  {
    this->currentPathTagName_ = "(unspecified path name)";
  }
  else
  {
    this->currentPathTagName_ = newPathTagName;
  }

  RCLCPP_INFO(this->get_logger(), "PUSH_PATH PATH EXITING");
  this->logStateString();
  RCLCPP_INFO(this->get_logger(), "odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::popPath(int popCount, bool keepPreviousPath)
{
  RCLCPP_INFO(this->get_logger(), "odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  RCLCPP_INFO(this->get_logger(), "POP PATH ENTRY");
  this->logStateString();

  if (!keepPreviousPath)
  {
    baseTrajectory_.poses.clear();
  }

  while (popCount > 0 && !pathStack_.empty())
  {
    auto & stacked = pathStack_.back().path.poses;
    baseTrajectory_.poses.insert(baseTrajectory_.poses.begin(), stacked.begin(), stacked.end());
    pathStack_.pop_back();
    popCount--;

    RCLCPP_INFO(this->get_logger(), "POP PATH Iteration ");
    this->logStateString();
  }

  RCLCPP_INFO(this->get_logger(), "POP PATH EXITING");
  this->logStateString();
  RCLCPP_INFO(this->get_logger(), "odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::logStateString()
{
  RCLCPP_INFO(this->get_logger(), "--- odom tracker state ---");
  RCLCPP_INFO(this->get_logger(), " - stacked paths count: %ld", pathStack_.size());
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    " - [STACK-HEAD active path '" << currentPathTagName_ << "' size: " << baseTrajectory_.poses.size() << "]");
  int i = 0;
  for (auto & p : pathStack_ | boost::adaptors::reversed)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " - p " << i << "[" << p.path.header.stamp.sec << "." << p.path.header.stamp.nanosec << "]["
              << p.pathTagName << "], size: " << p.path.poses.size());
    i++;
  }
  RCLCPP_INFO(this->get_logger(), "---");
}

void OdomTracker::clearPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  baseTrajectory_.poses.clear();

  rtPublishPaths(this->now());
  this->logStateString();
  this->updateAggregatedStackPath();
}

void OdomTracker::setStartPoint(const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] set current path starting point: " << pose);
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

void OdomTracker::setStartPoint(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] set current path starting point: " << pose);
  geometry_msgs::msg::PoseStamped posestamped;
  posestamped.header.frame_id = this->odomFrame_;
  posestamped.header.stamp = this->now();
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

nav_msgs::msg::Path OdomTracker::getPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  return this->baseTrajectory_;
}

/**
 ******************************************************************************************************************
 * rtPublishPaths()
 ******************************************************************************************************************
 */
void OdomTracker::rtPublishPaths(rclcpp::Time timestamp)
{
  baseTrajectory_.header.stamp = timestamp;
  baseTrajectory_.header.frame_id = this->odomFrame_;
  aggregatedStackPathMsg_.header.stamp = timestamp;
  aggregatedStackPathMsg_.header.frame_id = this->odomFrame_;

  if (robotBasePathPub_ && robotBasePathPub_->trylock())
  {
    robotBasePathPub_->msg_ = baseTrajectory_;
    robotBasePathPub_->unlockAndPublish();
  }

  if (robotBasePathStackedPub_ && robotBasePathStackedPub_->trylock())
  {
    robotBasePathStackedPub_->msg_ = aggregatedStackPathMsg_;
    robotBasePathStackedPub_->unlockAndPublish();
  }
}

void OdomTracker::updateAggregatedStackPath()
{
  aggregatedStackPathMsg_.poses.clear();
  for (auto & p : pathStack_)
  {
    aggregatedStackPathMsg_.poses.insert(
      aggregatedStackPathMsg_.poses.end(), p.path.poses.begin(), p.path.poses.end());
  }

  aggregatedStackPathMsg_.header.frame_id = this->odomFrame_;
}

/**
 ******************************************************************************************************************
 * updateBackward()
 ******************************************************************************************************************
 */
bool OdomTracker::updateClearPath(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped base_pose;

  base_pose.pose = odom.pose.pose;
  base_pose.header = odom.header;
  baseTrajectory_.header = odom.header;

  bool acceptBackward = false;
  bool clearingError = false;
  bool finished = false;

  while (!finished)
  {
    if (baseTrajectory_.poses.size() <= 1)
    {
      acceptBackward = false;
      finished = true;
    }
    else
    {
      auto & carrotPose = baseTrajectory_.poses.back().pose;
      const geometry_msgs::msg::Point & carrotPoint = carrotPose.position;
      double carrotAngle = tf2::getYaw(carrotPose.orientation);

      auto & currePose = base_pose.pose;
      const geometry_msgs::msg::Point & currePoint = currePose.position;
      double currentAngle = tf2::getYaw(currePose.orientation);

      double lastpointdist = p2pDistance(carrotPoint, currePoint);
      double goalAngleOffset = fabs(angles::shortest_angular_distance(carrotAngle, currentAngle));

      acceptBackward = !baseTrajectory_.poses.empty() && lastpointdist < clearPointDistanceThreshold_ &&
                       goalAngleOffset < clearAngularDistanceThreshold_;

      clearingError = lastpointdist > 2 * clearPointDistanceThreshold_;
      RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "[OdomTracker] clearing (accepted: " << acceptBackward << ") linerr: " << lastpointdist
                                             << ", anglerr: " << goalAngleOffset);
    }

    if (acceptBackward && baseTrajectory_.poses.size() > 1)
    {
      baseTrajectory_.poses.pop_back();
    }
    else if (clearingError)
    {
      finished = true;
      RCLCPP_WARN(this->get_logger(), "[OdomTracker] Incorrect odom clearing motion.");
    }
    else
    {
      finished = true;
    }
  }

  return acceptBackward;
}
/**
 ******************************************************************************************************************
 * updateRecordPath()
 ******************************************************************************************************************
 */
bool OdomTracker::updateRecordPath(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped base_pose;

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
    const auto & prevPose = baseTrajectory_.poses.back().pose;
    const geometry_msgs::msg::Point & prevPoint = prevPose.position;
    double prevAngle = tf2::getYaw(prevPose.orientation);

    const geometry_msgs::msg::Point & currePoint = base_pose.pose.position;
    double currentAngle = tf2::getYaw(base_pose.pose.orientation);

    dist = p2pDistance(prevPoint, currePoint);
    double goalAngleOffset = fabs(angles::shortest_angular_distance(prevAngle, currentAngle));

    if (dist > recordPointDistanceThreshold_ || goalAngleOffset > recordAngularDistanceThreshold_)
    {
      enqueueOdomMessage = true;
    }
    else
    {
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
void OdomTracker::reconfigCB(::move_base_z_client_plugin::OdomTrackerConfig & config, uint32_t level)
{
  (void)level;
  RCLCPP_INFO(this->get_logger(), "[OdomTracker] reconfigure Request");
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
void OdomTracker::processOdometryMessage(const nav_msgs::msg::Odometry & odom)
{
  std::lock_guard<std::mutex> lock(m_mutex_);

  if (workingMode_ == WorkingMode::RECORD_PATH)
  {
    updateRecordPath(odom);
  }
  else if (workingMode_ == WorkingMode::CLEAR_PATH)
  {
    updateClearPath(odom);
  }

  if (publishMessages)
  {
    rtPublishPaths(rclcpp::Time(odom.header.stamp));
  }
}
}  // namespace odom_tracker
}  // namespace cl_move_base_z