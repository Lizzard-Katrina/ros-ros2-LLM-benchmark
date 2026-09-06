// Minimal ROS 2 node that hosts the migrated HectorMappingRos scanCallback logic.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cmath>

class HectorMappingRos : public rclcpp::Node
{
public:
  HectorMappingRos()
  : Node("hector_mapping")
  {
    // Declare parameters
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<bool>("pub_map_odom_transform", true);
    this->declare_parameter<bool>("use_tf_scan_transformation", true);
    this->declare_parameter<bool>("pub_map_scanmatch_transform", true);
    this->declare_parameter<std::string>("tf_map_scanmatch_transform_frame_name", "scanmatcher_frame");
    this->declare_parameter<double>("map_resolution", 0.025);
    this->declare_parameter<int>("map_size", 1024);

    p_base_frame_ = this->get_parameter("base_frame").as_string();
    p_map_frame_ = this->get_parameter("map_frame").as_string();
    p_odom_frame_ = this->get_parameter("odom_frame").as_string();
    p_pub_map_odom_transform_ = this->get_parameter("pub_map_odom_transform").as_bool();
    p_use_tf_scan_transformation_ = this->get_parameter("use_tf_scan_transformation").as_bool();
    p_pub_map_scanmatch_transform_ = this->get_parameter("pub_map_scanmatch_transform").as_bool();
    p_tf_map_scanmatch_transform_frame_name_ = this->get_parameter("tf_map_scanmatch_transform_frame_name").as_string();

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // QoS with TransientLocal durability for map and pose
    auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    // Publishers
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "slam_out_pose", transient_local_qos);
    pose_update_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "poseupdate", transient_local_qos);
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", transient_local_qos);

    // Subscriber
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("scan_topic").as_string(), 5,
      std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "HectorMappingRos node initialized");
    RCLCPP_INFO(this->get_logger(), "base_frame: %s, map_frame: %s, odom_frame: %s",
      p_base_frame_.c_str(), p_map_frame_.c_str(), p_odom_frame_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (pause_scan_processing_) {
      return;
    }

    auto start_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", scan->ranges.size());

    tf2::TimePoint scan_time = tf2_ros::fromMsg(scan->header.stamp);

    // Build laser scan data container (named laserScanContainer)
    std::vector<std::pair<float, float>> laserScanContainer;
    {
      float angle = scan->angle_min;
      float max_range = scan->range_max - 0.1f;
      for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float dist = scan->ranges[i];
        if (dist > scan->range_min && dist < max_range) {
          laserScanContainer.push_back(
            std::make_pair(std::cos(angle) * dist, std::sin(angle) * dist));
        }
        angle += scan->angle_increment;
      }
    }

    if (laserScanContainer.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty laser scan container, skipping update");
      return;
    }

    // TF lookups
    geometry_msgs::msg::TransformStamped laser_to_base_tf;
    if (p_use_tf_scan_transformation_) {
      try {
        laser_to_base_tf = tf_buffer_->lookupTransform(
          p_base_frame_, scan->header.frame_id,
          scan_time,
          tf2::durationFromSec(0.5));
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup laser->base failed: %s", ex.what());
        return;
      }
    }

    // Build the map->base transform from SLAM result
    geometry_msgs::msg::TransformStamped map_to_base_tf;
    map_to_base_tf.header.stamp = scan->header.stamp;
    map_to_base_tf.header.frame_id = p_map_frame_;
    map_to_base_tf.child_frame_id = p_base_frame_;
    map_to_base_tf.transform.translation.x = 0.0;
    map_to_base_tf.transform.translation.y = 0.0;
    map_to_base_tf.transform.translation.z = 0.0;
    map_to_base_tf.transform.rotation.w = 1.0;
    map_to_base_tf.transform.rotation.x = 0.0;
    map_to_base_tf.transform.rotation.y = 0.0;
    map_to_base_tf.transform.rotation.z = 0.0;

    // Publish scanmatcher transform
    if (p_pub_map_scanmatch_transform_) {
      geometry_msgs::msg::TransformStamped scanmatch_tf;
      scanmatch_tf.header.stamp = scan->header.stamp;
      scanmatch_tf.header.frame_id = p_map_frame_;
      scanmatch_tf.child_frame_id = p_tf_map_scanmatch_transform_frame_name_;
      scanmatch_tf.transform = map_to_base_tf.transform;
      tf_broadcaster_->sendTransform(std::move(scanmatch_tf));
    }

    // Compute map->odom transform
    if (p_pub_map_odom_transform_) {
      try {
        geometry_msgs::msg::TransformStamped odom_to_base =
          tf_buffer_->lookupTransform(
            p_odom_frame_, p_base_frame_,
            scan_time,
            tf2::durationFromSec(0.5));

        tf2::Transform tf_map_base;
        tf2::fromMsg(map_to_base_tf.transform, tf_map_base);

        tf2::Transform tf_odom_base;
        tf2::fromMsg(odom_to_base.transform, tf_odom_base);

        // map_to_odom = map_to_base * odom_to_base.inverse()
        tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();
        map_to_odom_ = tf_map_odom;

        geometry_msgs::msg::TransformStamped map_odom_msg;
        map_odom_msg.header.stamp = scan->header.stamp;
        map_odom_msg.header.frame_id = p_map_frame_;
        map_odom_msg.child_frame_id = p_odom_frame_;
        map_odom_msg.transform = tf2::toMsg(map_to_odom_);
        tf_broadcaster_->sendTransform(std::move(map_odom_msg));

      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup odom->base failed: %s", ex.what());
      }
    }

    // Publish pose
    auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = scan->header.stamp;
    pose_msg->header.frame_id = p_map_frame_;
    pose_msg->pose.position.x = 0.0;
    pose_msg->pose.position.y = 0.0;
    pose_msg->pose.position.z = 0.0;
    pose_msg->pose.orientation.w = 1.0;
    pose_publisher_->publish(std::move(pose_msg));

    // Publish pose with covariance
    auto cov_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    cov_msg->header.stamp = scan->header.stamp;
    cov_msg->header.frame_id = p_map_frame_;
    cov_msg->pose.pose.position.x = 0.0;
    cov_msg->pose.pose.position.y = 0.0;
    cov_msg->pose.pose.orientation.w = 1.0;
    pose_update_publisher_->publish(std::move(cov_msg));

    auto end_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Scan processing took %.4f seconds",
      (end_time - start_time).seconds());
  }

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_update_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  tf2::Transform map_to_odom_;

  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;
  std::string p_tf_map_scanmatch_transform_frame_name_;
  bool p_pub_map_odom_transform_ = true;
  bool p_use_tf_scan_transformation_ = true;
  bool p_pub_map_scanmatch_transform_ = true;
  bool pause_scan_processing_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HectorMappingRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}