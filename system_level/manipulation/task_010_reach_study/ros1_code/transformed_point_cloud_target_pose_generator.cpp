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

// TODO: Generate and transform target poses from the source point cloud to the target frame.
  // - Generate initial poses using the base PointCloudTargetPoseGenerator.
  // - Use tf2_ros to look up the transform between 'points_frame_' and 'target_frame_'.
  // - Apply a 3.0s timeout for the transform lookup.
  // - Use 'tf2::transformToEigen' for conversion and apply the transform to all poses.
  // - Return the set of transformed Isometry3d poses.
//END OF TODO
}  // namespace reach_ros
