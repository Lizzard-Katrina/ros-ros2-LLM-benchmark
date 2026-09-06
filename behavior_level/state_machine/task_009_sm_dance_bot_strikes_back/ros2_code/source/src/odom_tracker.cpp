/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 *   Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <angles/angles.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <task_009_sm_dance_bot_strikes_back/odom_tracker.h>

namespace cl_move_base_z
{
namespace odom_tracker
{
OdomTracker::OdomTracker(std::string odomTopicName, std::string odomFrame)
  : rclcpp::Node("odom_tracker")
{
  workingMode_ = WorkingMode::RECORD_PATH;
  publishMessages = true;
  subscribeToOdometryTopic_ = true;
  this->odomFrame_ = odomFrame;

  RCLCPP_WARN(this->get_logger(), "Initializing Odometry Tracker");

  // Declare parameters with defaults
  this->declare_parameter("odom_frame", odomFrame);
  this->declare_parameter("record_point_distance_threshold", 0.005);
  this->declare_parameter("record_angular_distance_threshold", 0.1);
  this->declare_parameter("clear_point_distance_threshold", 0.05);
  this->declare_parameter("clear_angular_distance_threshold", 0.1);

  // Get parameters
  this->get_parameter("odom_frame", this->odomFrame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] odomFrame: " << this->odomFrame_);

  this->get_parameter("record_point_distance_threshold", recordPointDistanceThreshold_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] record_point_distance_threshold: " << recordPointDistanceThreshold_);

  this->get_parameter("record_angular_distance_threshold", recordAngularDistanceThreshold_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] record_angular_distance_threshold: " << recordAngularDistanceThreshold_);

  this->get_parameter("clear_point_distance_threshold", clearPointDistanceThreshold_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] clear_point_distance_threshold: " << clearPointDistanceThreshold_);

  this->get_parameter("clear_angular_distance_threshold", clearAngularDistanceThreshold_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] clear_angular_distance_threshold: " << clearAngularDistanceThreshold_);

  if (this->subscribeToOdometryTopic_)
  {
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odomTopicName, 1,
        std::bind(&OdomTracker::processOdometryMessage, this, std::placeholders::_1));
  }

  // Create realtime publishers using get_node_base_interface() + get_node_topics_interface()
  robotBasePathPub_ = std::make_shared<RealtimePublisher<nav_msgs::msg::Path>>(
      this->get_node_base_interface(), this->get_node_topics_interface(),
      "odom_tracker_path", 1);
  robotBasePathStackedPub_ = std::make_shared<RealtimePublisher<nav_msgs::msg::Path>>(
      this->get_node_base_interface(), this->get_node_topics_interface(),
      "odom_tracker_stacked_path", 1);
}

void OdomTracker::setWorkingMode(WorkingMode workingMode)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO(this->get_logger(), "[OdomTracker] setting working mode to: %d", (uint8_t)workingMode);
  workingMode_ = workingMode;
}

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
    auto &stacked = pathStack_.back().path.poses;
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
  RCLCPP_INFO_STREAM(this->get_logger(), " - [STACK-HEAD active path '" << currentPathTagName_ << "' size: " << baseTrajectory_.poses.size() << "]");
  int i = 0;
  for (auto it = pathStack_.rbegin(); it != pathStack_.rend(); ++it)
  {
    auto &p = *it;
    RCLCPP_INFO_STREAM(this->get_logger(), " - p " << i << "[" << p.path.header.stamp.sec << "." << p.path.header.stamp.nanosec << "][" << p.pathTagName << "], size: " << p.path.poses.size());
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

void OdomTracker::setStartPoint(const geometry_msgs::msg::PoseStamped &pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] set current path starting point: " << pose.pose.position.x << ", " << pose.pose.position.y);
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

void OdomTracker::setStartPoint(const geometry_msgs::msg::Pose &pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(this->get_logger(), "[OdomTracker] set current path starting point: " << pose.position.x << ", " << pose.position.y);
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

void OdomTracker::rtPublishPaths(rclcpp::Time timestamp)
{
  if (robotBasePathPub_->trylock())
  {
    nav_msgs::msg::Path &msg = robotBasePathPub_->msg_;
    msg = baseTrajectory_;
    msg.header.stamp = timestamp;
    robotBasePathPub_->unlockAndPublish();
  }

  if (robotBasePathStackedPub_->trylock())
  {
    nav_msgs::msg::Path &msg = robotBasePathStackedPub_->msg_;
    msg = aggregatedStackPathMsg_;
    msg.header.stamp = timestamp;
    robotBasePathStackedPub_->unlockAndPublish();
  }
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

bool OdomTracker::updateClearPath(const nav_msgs::msg::Odometry &odom)
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
      auto &carrotPose = baseTrajectory_.poses.back().pose;
      const geometry_msgs::msg::Point &carrotPoint = carrotPose.position;
      double carrotAngle = tf2::getYaw(carrotPose.orientation);

      auto &currePose = base_pose.pose;
      const geometry_msgs::msg::Point &currePoint = currePose.position;
      double currentAngle = tf2::getYaw(currePose.orientation);

      double lastpointdist = p2pDistance(carrotPoint, currePoint);
      double goalAngleOffset = fabs(angles::shortest_angular_distance(carrotAngle, currentAngle));

      acceptBackward = !baseTrajectory_.poses.empty() && lastpointdist < clearPointDistanceThreshold_ &&
                       goalAngleOffset < clearAngularDistanceThreshold_;

      clearingError = lastpointdist > 2 * clearPointDistanceThreshold_;
      RCLCPP_DEBUG_STREAM(this->get_logger(), "[OdomTracker] clearing (accepted: " << acceptBackward << ") linerr: " << lastpointdist
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

bool OdomTracker::updateRecordPath(const nav_msgs::msg::Odometry &odom)
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
    const auto &prevPose = baseTrajectory_.poses.back().pose;
    const geometry_msgs::msg::Point &prevPoint = prevPose.position;
    double prevAngle = tf2::getYaw(prevPose.orientation);

    const geometry_msgs::msg::Point &currePoint = base_pose.pose.position;
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

void OdomTracker::processOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  std::lock_guard<std::mutex> lock(m_mutex_);

  if (workingMode_ == WorkingMode::RECORD_PATH)
  {
    updateRecordPath(*odom);
  }
  else if (workingMode_ == WorkingMode::CLEAR_PATH)
  {
    updateClearPath(*odom);
  }

  if (publishMessages)
  {
    rtPublishPaths(rclcpp::Time(odom->header.stamp));
  }
}

}  // namespace odom_tracker
}  // namespace cl_move_base_z