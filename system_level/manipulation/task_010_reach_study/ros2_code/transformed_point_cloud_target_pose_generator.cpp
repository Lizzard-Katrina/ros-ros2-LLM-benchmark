#include <reach_ros/target_pose_generator/transformed_point_cloud_target_pose_generator.h>
#include <reach_ros/utils.h>

#include <reach/plugin_utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

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

std::vector<Eigen::Isometry3d> TransformedPointCloudTargetPoseGenerator::generatePoses() const
{
  std::vector<Eigen::Isometry3d> poses = reach::PointCloudTargetPoseGenerator::generatePoses();

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(target_frame_, points_frame_, tf2::Time(0), tf2::Duration(3.0));
  }
  catch (tf2::TransformException& ex)
  {
    throw std::runtime_error("Failed to lookup transform from " + points_frame_ + " to " + target_frame_);
  }

  Eigen::Isometry3d transform_eigen;
  tf2::fromMsg(transform.transform, transform_eigen);

  for (auto& pose : poses)
  {
    pose = transform_eigen * pose;
  }

  return poses;
}

}  // namespace reach_ros