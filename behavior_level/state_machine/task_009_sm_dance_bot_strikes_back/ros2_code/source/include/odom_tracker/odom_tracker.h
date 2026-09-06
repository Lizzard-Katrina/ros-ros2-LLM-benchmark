#ifndef ODOM_TRACKER__ODOM_TRACKER_H_
#define ODOM_TRACKER__ODOM_TRACKER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <functional>

namespace cl_move_base_z
{
namespace odom_tracker
{

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
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>> robotBasePathPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>> robotBasePathStackedPub_;
};

}  // namespace odom_tracker
}  // namespace cl_move_base_z

#endif  // ODOM_TRACKER__ODOM_TRACKER_H_