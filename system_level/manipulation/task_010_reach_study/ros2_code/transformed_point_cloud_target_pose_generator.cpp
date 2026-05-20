#include <reach_ros/target_pose_generator/transformed_point_cloud_target_pose_generator.h>
#include <reach_ros/utils.h>

#include <reach/plugin_utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace reach_ros
{
TransformedPointCloudTargetPoseGenerator::TransformedPointCloudTargetPoseGenerator(std::string filename,
                                                                                   std::string points_frame,
                                                                                   std::string target_frame)
  : reach::PointCloudTargetPoseGenerator(filename)
  , points_frame_(std::move(points_frame))
  , target_frame_(std::move(target_frame))
{
}

std::vector<Eigen::Isometry3d> TransformedPointCloudTargetPoseGenerator::generate() const
{
  auto poses = reach::PointCloudTargetPoseGenerator::generate();

  auto node = rclcpp::Node::make_shared("tf_lookup_node");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform(target_frame_, points_frame_, tf2::TimePointZero, tf2::durationFromSec(3.0));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Transform lookup failed: %s", ex.what());
    return {};
  }

  Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
  std::vector<Eigen::Isometry3d> transformed_poses;
  transformed_poses.reserve(poses.size());

  for (const auto& pose : poses)
  {
    transformed_poses.push_back(transform * pose);
  }

  return transformed_poses;
}

}  // namespace reach_ros