// ROS2 Radar Ego Velocity Estimator Node
// Minimal node that subscribes to PointCloud2 and publishes TwistWithCovarianceStamped

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <mutex>
#include <string>

namespace reve {

class RadarEgoVelocityEstimatorRos : public rclcpp::Node
{
public:
  RadarEgoVelocityEstimatorRos()
    : Node("radar_ego_velocity_estimator"), trigger_stamp(0, 0, RCL_ROS_TIME)
  {
    this->declare_parameter<bool>("run_without_trigger", true);
    this->get_parameter("run_without_trigger", run_without_trigger);

    if (run_without_trigger)
      RCLCPP_WARN(this->get_logger(), "%s Running without radar trigger", kPrefix.c_str());

    sub_radar_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensor_platform/radar/scan", 50,
      std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarScan, this, std::placeholders::_1));

    sub_radar_trigger_ = this->create_subscription<std_msgs::msg::Header>(
      "/sensor_platform/radar/trigger", 50,
      std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this, std::placeholders::_1));

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist", 5);
  }

  void processRadarData(const sensor_msgs::msg::PointCloud2& radar_scan,
                        const rclcpp::Time& trigger_stamp_val)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = trigger_stamp_val;
    msg.header.frame_id = (radar_scan.header.frame_id.empty()) ? "radar" : radar_scan.header.frame_id;
    // In a real implementation, the estimator would fill in velocity values
    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    pub_twist_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%s Published twist estimate", kPrefix.c_str());
  }

  void callbackRadarScan(const sensor_msgs::msg::PointCloud2::SharedPtr radar_scan_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (run_without_trigger)
    {
      rclcpp::Time stamp(radar_scan_msg->header.stamp);
      if (radar_scan_msg->header.stamp.sec == 0)
      {
        stamp = this->now();
        RCLCPP_WARN(this->get_logger(), "%s Radar scan timestamp is zero, using this->now()", kPrefix.c_str());
      }
      processRadarData(*radar_scan_msg, stamp);
    }
    else
    {
      if (trigger_stamp.nanoseconds() > 0)
      {
        rclcpp::Time stamp_to_use = trigger_stamp;

        if (radar_scan_msg->header.stamp.sec == 0)
        {
          stamp_to_use = this->now();
          RCLCPP_WARN(this->get_logger(), "%s Radar scan timestamp is zero, using this->now()", kPrefix.c_str());
        }

        processRadarData(*radar_scan_msg, stamp_to_use);

        // Reset trigger_stamp after consumption to prevent stale data
        trigger_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "%s Waiting for radar trigger...", kPrefix.c_str());
      }
    }
  }

  void callbackRadarTrigger(const std_msgs::msg::Header::SharedPtr trigger_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trigger_stamp = rclcpp::Time(trigger_msg->stamp);
  }

private:
  const std::string kPrefix{"[RadarEgoVelocityEstimatorRos]: "};
  std::mutex mutex_;
  rclcpp::Time trigger_stamp;
  bool run_without_trigger{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_radar_scan_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_radar_trigger_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
};

}  // namespace reve

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<reve::RadarEgoVelocityEstimatorRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}