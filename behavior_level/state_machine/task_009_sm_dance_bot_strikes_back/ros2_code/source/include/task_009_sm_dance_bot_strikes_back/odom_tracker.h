#ifndef TASK_009_ODOM_TRACKER_H
#define TASK_009_ODOM_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <atomic>

namespace cl_move_base_z
{
namespace odom_tracker
{

/// @brief A drop-in shim that mimics the realtime_tools RealtimePublisher API
/// on top of a plain rclcpp::Publisher so that the migration keeps the same
/// call-sites (trylock / msg_ / unlockAndPublish).
template <typename MessageT>
class RealtimePublisher
{
public:
  /// Construct from a node pointer
  RealtimePublisher(
      rclcpp::Node * node,
      const std::string & topic,
      size_t qos_history_depth)
  {
    pub_ = node->create_publisher<MessageT>(
        topic,
        rclcpp::QoS(static_cast<int>(qos_history_depth)));
  }

  /// Overload accepting NodeBaseInterface::SharedPtr
  RealtimePublisher(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr /*node_base*/,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      const std::string & topic,
      size_t qos_history_depth)
  {
    pub_ = rclcpp::create_publisher<MessageT>(
        node_topics,
        topic,
        rclcpp::QoS(static_cast<int>(qos_history_depth)));
  }

  bool trylock()
  {
    bool expected = false;
    return locked_.compare_exchange_strong(expected, true);
  }

  void unlockAndPublish()
  {
    pub_->publish(msg_);
    locked_.store(false);
  }

  MessageT msg_;

private:
  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  std::atomic<bool> locked_{false};
};

enum class WorkingMode : uint8_t
{
  IDLE = 0,
  RECORD_PATH = 1,
  CLEAR_PATH = 2
};

struct PathInfo
{
  nav_msgs::msg::Path path;
  std::string pathTagName;
};

class OdomTracker : public rclcpp::Node
{
public:
  OdomTracker(std::string odomTopicName = "odom", std::string odomFrame = "odom");

  void setWorkingMode(WorkingMode workingMode);
  void setPublishMessages(bool value);
  void pushPath(std::string newPathTagName = "");
  void popPath(int popCount = 1, bool keepPreviousPath = false);
  void clearPath();
  void setStartPoint(const geometry_msgs::msg::PoseStamped &pose);
  void setStartPoint(const geometry_msgs::msg::Pose &pose);
  nav_msgs::msg::Path getPath();
  void logStateString();

protected:
  void rtPublishPaths(rclcpp::Time timestamp);
  void updateAggregatedStackPath();
  bool updateClearPath(const nav_msgs::msg::Odometry &odom);
  bool updateRecordPath(const nav_msgs::msg::Odometry &odom);
  void processOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr odom);

  static double p2pDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::mutex m_mutex_;
  WorkingMode workingMode_;
  bool publishMessages;
  bool subscribeToOdometryTopic_;

  std::string odomFrame_;
  std::string currentPathTagName_;

  double recordPointDistanceThreshold_;
  double recordAngularDistanceThreshold_;
  double clearPointDistanceThreshold_;
  double clearAngularDistanceThreshold_;

  nav_msgs::msg::Path baseTrajectory_;
  nav_msgs::msg::Path aggregatedStackPathMsg_;
  std::vector<PathInfo> pathStack_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  std::shared_ptr<RealtimePublisher<nav_msgs::msg::Path>> robotBasePathPub_;
  std::shared_ptr<RealtimePublisher<nav_msgs::msg::Path>> robotBasePathStackedPub_;
};

}  // namespace odom_tracker
}  // namespace cl_move_base_z

#endif  // TASK_009_ODOM_TRACKER_H