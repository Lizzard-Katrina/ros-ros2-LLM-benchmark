# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: manipulability_moveit.cpp
----------------------------
/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/evaluation/manipulability_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>
#include <numeric>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <yaml-cpp/yaml.h>

static std::vector<Eigen::Index> getJacobianRowSubset(const YAML::Node& config, const std::string& key = "jacobian_row_"
                                                                                                         "subset")
{
  std::vector<Eigen::Index> jacobian_row_subset;

  const YAML::Node& jrs_config = config[key];
  if (jrs_config.IsDefined())
  {
    std::set<Eigen::Index> subset_rows;
    for (auto it = jrs_config.begin(); it != jrs_config.end(); ++it)
    {
      int row = (*it).as<Eigen::Index>();
      if (row < 0 || row >= 6)
      {
        std::stringstream ss;
        ss << "Invalid Jacobian row subset index provided: " << row << ". Must be on interval [0, 6)";
        throw std::runtime_error(ss.str());
      }

      subset_rows.insert(row);
    }

    if (subset_rows.empty())
      throw std::runtime_error("Jacobian row subset is empty");

    std::copy(subset_rows.begin(), subset_rows.end(), std::back_inserter(jacobian_row_subset));
  }
  else
  {
    jacobian_row_subset.resize(6);
    std::iota(jacobian_row_subset.begin(), jacobian_row_subset.end(), 0);
  }

  return jacobian_row_subset;
}

static std::vector<std::string> getExcludedLinks(const YAML::Node& config, const std::string& key = "excluded_links")
{
  try
  {
    return reach::get<std::vector<std::string>>(config, key);
  }
  catch (const std::exception& ex)
  {
    return {};
  }
}

namespace reach_ros
{
namespace evaluation
{
double recurse(const moveit::core::JointModel* joint, const moveit::core::RobotState& state,
               const std::string& tcp_frame)
{
  const moveit::core::LinkModel* child_link = joint->getChildLinkModel();
  if (child_link->getName() == tcp_frame)
    return child_link->getJointOriginTransform().translation().norm();

  std::vector<const moveit::core::JointModel*> children = child_link->getChildJointModels();

  // Anything other than 1 child suggests that there has been a branch of joints
  if (children.size() != 1)
    throw std::runtime_error("The robot model urdf has branching joints after the last active joint. This "
                             "configuration is currently unsupported");

  switch (children.at(0)->getType())
  {
    case moveit::core::JointModel::FIXED:
      break;
    default:
      throw std::runtime_error("The robot model contains non-fixed joints after the last active joint. This "
                               "configuration is currently unsupported");
  }

  double d = child_link->getJointOriginTransform().translation().norm();
  d += recurse(children.at(0), state, tcp_frame);
  return d;
}

ManipulabilityMoveIt::ManipulabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset)
  : model_(std::move(model))
  , jmg_(model_->getJointModelGroup(planning_group))
  , jacobian_row_subset_(std::move(jacobian_row_subset))
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group pointer");
}

double ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);

  // Take the subset of joints in the joint model group out of the input pose
  std::vector<double> pose_subset = reach::extractSubset(pose, jmg_->getActiveJointModelNames());
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Extract the partial jacobian
  if (jacobian_row_subset_.size() < 6)
  {
    Eigen::MatrixXd partial_jacobian(jacobian_row_subset_.size(), jacobian.cols());
    for (std::size_t i = 0; i < jacobian_row_subset_.size(); ++i)
    {
      partial_jacobian.row(i) = jacobian.row(jacobian_row_subset_[i]);
    }

    jacobian = partial_jacobian;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return calculateScore(singular_values);
}

double ManipulabilityMoveIt::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.array().prod();
}

reach::Evaluator::ConstPtr ManipulabilityMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityMoveIt>(model, planning_group, jacobian_row_subset);
}

double ManipulabilityRatio::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.minCoeff() / jacobian_singular_values.maxCoeff();
}

reach::Evaluator::ConstPtr ManipulabilityRatioFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityRatio>(model, planning_group, jacobian_row_subset);
}

ManipulabilityScaled::ManipulabilityScaled(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset,
                                           std::vector<std::string> excluded_links)
  : ManipulabilityMoveIt(model, planning_group, jacobian_row_subset), excluded_links_(std::move(excluded_links))
{
  characteristic_length_ = calculateCharacteristicLength(model_, jmg_, excluded_links_);
}

double ManipulabilityScaled::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  if (std::abs(characteristic_length_) < std::numeric_limits<double>::epsilon())
    throw std::runtime_error("The model must have a non-zero characteristic length");

  return ManipulabilityMoveIt::calculateScore(jacobian_singular_values) / characteristic_length_;
}

reach::Evaluator::ConstPtr ManipulabilityScaledFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);
  std::vector<std::string> excluded_links = getExcludedLinks(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityScaled>(model, planning_group, jacobian_row_subset, excluded_links);
}

double calculateCharacteristicLength(moveit::core::RobotModelConstPtr model, const moveit::core::JointModelGroup* jmg,
                                     const std::vector<std::string>& excluded_links)
{
// TODO: Implement the manipulability scoring logic for a given robot configuration.
  // - Update the robot state with the provided joint positions.
  // - Retrieve the full Jacobian matrix for the current joint model group.
  // - If 'jacobian_row_subset_' is active (size < 6), extract the corresponding rows to form a partial Jacobian.
  // - Compute the singular values using SVD and return the score via 'calculateScore'.
  // - Note: Use Eigen-based matrix operations for efficiency.
//END OF TODO
}

}  // namespace evaluation
}  // namespace reach_ros

----------------------------

FILE_PATH: moveit_ik_solver.cpp
----------------------------
/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/ik/moveit_ik_solver.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
T clamp(const T& val, const T& low, const T& high)
{
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace reach_ros
{
namespace ik
{
std::string MoveItIKSolver::COLLISION_OBJECT_NAME = "reach_object";

MoveItIKSolver::MoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                               double dist_threshold)
  : model_(model), jmg_(model_->getJointModelGroup(planning_group)), distance_threshold_(dist_threshold)
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group for planning group '" + planning_group + "'");
  if (!jmg_->getSolverInstance())
    throw std::runtime_error("No kinematics solver instantiated for planning group '" + planning_group +
                             "'. Check that the 'kinematics.yaml' file was loaded as a parameter");

  scene_.reset(new planning_scene::PlanningScene(model_));

  ros::NodeHandle nh;
  scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  moveit_msgs::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
}

std::vector<std::vector<double>> MoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                         const std::map<std::string, double>& seed) const
{
// TODO: Implement the complete IK solving logic for the reach study.
  // [STYLE CONSTRAINT]: You MUST use the 'state.setFromIK' method.
  // [STYLE CONSTRAINT]: You MUST use 'boost::bind' to pass the 'isIKSolutionValid' callback.
  // - Initialize the robot state and set the initial joint positions using the provided seed.
  // - Solve Inverse Kinematics for the 'target' isometry.
  // - You MUST use 'isIKSolutionValid' as the state validity callback.
  // - Return the successful joint solution as a vector of vectors, or an empty vector if failed.
  // - Ensure the robot state is updated before and after the IK call.
//END TODO
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);

  return (!colliding && !too_close);
}

std::vector<std::string> MoveItIKSolver::getJointNames() const
{
  return jmg_->getActiveJointModelNames();
}

void MoveItIKSolver::addCollisionMesh(const std::string& collision_mesh_filename,
                                      const std::string& collision_mesh_frame)
{
  // Add the collision object to the planning scene
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename, collision_mesh_frame, COLLISION_OBJECT_NAME);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  moveit_msgs::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
}

void MoveItIKSolver::setTouchLinks(const std::vector<std::string>& touch_links)
{
  scene_->getAllowedCollisionMatrixNonConst().setEntry(COLLISION_OBJECT_NAME, touch_links, true);
}

std::string MoveItIKSolver::getKinematicBaseFrame() const
{
  return jmg_->getSolverInstance()->getBaseFrame();
}

reach::IKSolver::ConstPtr MoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto ik_solver = std::make_shared<MoveItIKSolver>(model, planning_group, dist_threshold);

  // Optionally add a collision mesh
  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           ik_solver->getKinematicBaseFrame();

    ik_solver->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  // Optionally add touch links
  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    ik_solver->setTouchLinks(touch_links);
  }

  return ik_solver;
}

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver(moveit::core::RobotModelConstPtr model,
                                                     const std::string& planning_group, double dist_threshold,
                                                     double dt)
  : MoveItIKSolver(model, planning_group, dist_threshold), dt_(dt)
{
}

std::vector<std::vector<double>> DiscretizedMoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                                    const std::map<std::string, double>& seed) const
{
  // Calculate the number of discretizations necessary to achieve discretization angle
  const static int n_discretizations = int((2.0 * M_PI) / dt_);

  std::vector<std::vector<double>> solutions;
  solutions.reserve(n_discretizations);

  for (int i = 0; i < n_discretizations; ++i)
  {
    Eigen::Isometry3d discretized_target(target * Eigen::AngleAxisd(double(i) * dt_, Eigen::Vector3d::UnitZ()));
    std::vector<std::vector<double>> tmp_sols = MoveItIKSolver::solveIK(discretized_target, seed);

    if (!tmp_sols.empty())
      solutions.push_back(tmp_sols.front());
  }

  return solutions;
}

reach::IKSolver::ConstPtr DiscretizedMoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto dt = std::abs(reach::get<double>(config, "discretization_angle"));
  double clamped_dt = clamp<double>(dt, 0.0, M_PI);
  if (std::abs(dt - clamped_dt) > 1.0e-6)
  {
    std::cout << "Clamping discretization angle between 0 and pi; new value is " << clamped_dt;
  }
  dt = clamped_dt;

  auto ik_solver = std::make_shared<DiscretizedMoveItIKSolver>(model, planning_group, dist_threshold, dt);

  // Optionally add a collision mesh
  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           ik_solver->getKinematicBaseFrame();

    ik_solver->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    ik_solver->setTouchLinks(touch_links);
  }

  return ik_solver;
}

}  // namespace ik
}  // namespace reach_ros

----------------------------

FILE_PATH: transformed_point_cloud_target_pose_generator.cpp
----------------------------
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

----------------------------


# LLM Response

[FILENAME: manipulability_moveit.cpp]
/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/evaluation/manipulability_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>
#include <numeric>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <yaml-cpp/yaml.h>

static std::vector<Eigen::Index> getJacobianRowSubset(const YAML::Node& config, const std::string& key = "jacobian_row_"
                                                                                                         "subset")
{
  std::vector<Eigen::Index> jacobian_row_subset;

  const YAML::Node& jrs_config = config[key];
  if (jrs_config.IsDefined())
  {
    std::set<Eigen::Index> subset_rows;
    for (auto it = jrs_config.begin(); it != jrs_config.end(); ++it)
    {
      int row = (*it).as<Eigen::Index>();
      if (row < 0 || row >= 6)
      {
        std::stringstream ss;
        ss << "Invalid Jacobian row subset index provided: " << row << ". Must be on interval [0, 6)";
        throw std::runtime_error(ss.str());
      }

      subset_rows.insert(row);
    }

    if (subset_rows.empty())
      throw std::runtime_error("Jacobian row subset is empty");

    std::copy(subset_rows.begin(), subset_rows.end(), std::back_inserter(jacobian_row_subset));
  }
  else
  {
    jacobian_row_subset.resize(6);
    std::iota(jacobian_row_subset.begin(), jacobian_row_subset.end(), 0);
  }

  return jacobian_row_subset;
}

static std::vector<std::string> getExcludedLinks(const YAML::Node& config, const std::string& key = "excluded_links")
{
  try
  {
    return reach::get<std::vector<std::string>>(config, key);
  }
  catch (const std::exception& ex)
  {
    return {};
  }
}

namespace reach_ros
{
namespace evaluation
{
double recurse(const moveit::core::JointModel* joint, const moveit::core::RobotState& state,
               const std::string& tcp_frame)
{
  const moveit::core::LinkModel* child_link = joint->getChildLinkModel();
  if (child_link->getName() == tcp_frame)
    return child_link->getJointOriginTransform().translation().norm();

  std::vector<const moveit::core::JointModel*> children = child_link->getChildJointModels();

  if (children.size() != 1)
    throw std::runtime_error("The robot model urdf has branching joints after the last active joint. This "
                             "configuration is currently unsupported");

  switch (children.at(0)->getType())
  {
    case moveit::core::JointModel::FIXED:
      break;
    default:
      throw std::runtime_error("The robot model contains non-fixed joints after the last active joint. This "
                               "configuration is currently unsupported");
  }

  double d = child_link->getJointOriginTransform().translation().norm();
  d += recurse(children.at(0), state, tcp_frame);
  return d;
}

ManipulabilityMoveIt::ManipulabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset)
  : model_(std::move(model))
  , jmg_(model_->getJointModelGroup(planning_group))
  , jacobian_row_subset_(std::move(jacobian_row_subset))
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group pointer");
}

double ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  moveit::core::RobotState state(model_);

  std::vector<double> pose_subset = reach::extractSubset(pose, jmg_->getActiveJointModelNames());
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  if (jacobian_row_subset_.size() < 6)
  {
    Eigen::MatrixXd partial_jacobian(jacobian_row_subset_.size(), jacobian.cols());
    for (std::size_t i = 0; i < jacobian_row_subset_.size(); ++i)
    {
      partial_jacobian.row(i) = jacobian.row(jacobian_row_subset_[i]);
    }

    jacobian = partial_jacobian;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return calculateScore(singular_values);
}

double ManipulabilityMoveIt::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.array().prod();
}

reach::Evaluator::ConstPtr ManipulabilityMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityMoveIt>(model, planning_group, jacobian_row_subset);
}

double ManipulabilityRatio::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.minCoeff() / jacobian_singular_values.maxCoeff();
}

reach::Evaluator::ConstPtr ManipulabilityRatioFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityRatio>(model, planning_group, jacobian_row_subset);
}

ManipulabilityScaled::ManipulabilityScaled(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset,
                                           std::vector<std::string> excluded_links)
  : ManipulabilityMoveIt(model, planning_group, jacobian_row_subset), excluded_links_(std::move(excluded_links))
{
  characteristic_length_ = calculateCharacteristicLength(model_, jmg_, excluded_links_);
}

double ManipulabilityScaled::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  if (std::abs(characteristic_length_) < std::numeric_limits<double>::epsilon())
    throw std::runtime_error("The model must have a non-zero characteristic length");

  return ManipulabilityMoveIt::calculateScore(jacobian_singular_values) / characteristic_length_;
}

reach::Evaluator::ConstPtr ManipulabilityScaledFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);
  std::vector<std::string> excluded_links = getExcludedLinks(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityScaled>(model, planning_group, jacobian_row_subset, excluded_links);
}

double calculateCharacteristicLength(moveit::core::RobotModelConstPtr model, const moveit::core::JointModelGroup* jmg,
                                     const std::vector<std::string>& excluded_links)
{
  if (!model)
    throw std::runtime_error("Null robot model pointer provided");
  if (!jmg)
    throw std::runtime_error("Null joint model group pointer provided");

  const auto& active_joints = jmg->getActiveJointModels();
  if (active_joints.empty())
    throw std::runtime_error("The planning group has no active joints");

  const auto& link_models = jmg->getLinkModels();
  if (link_models.empty())
    throw std::runtime_error("The planning group has no links");

  std::set<std::string> excluded(excluded_links.begin(), excluded_links.end());

  const moveit::core::LinkModel* tcp_link = nullptr;
  for (auto it = link_models.rbegin(); it != link_models.rend(); ++it)
  {
    if (excluded.find((*it)->getName()) == excluded.end())
    {
      tcp_link = *it;
      break;
    }
  }

  if (tcp_link == nullptr)
    throw std::runtime_error("All planning group links are excluded; failed to determine TCP frame");

  const moveit::core::JointModel* last_active_joint = active_joints.back();
  const moveit::core::LinkModel* last_active_child = last_active_joint->getChildLinkModel();
  if (!last_active_child)
    throw std::runtime_error("Failed to determine child link for last active joint");

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  const std::string tcp_frame = tcp_link->getName();
  if (last_active_child->getName() == tcp_frame)
    return last_active_child->getJointOriginTransform().translation().norm();

  return recurse(last_active_joint, state, tcp_frame);
}

}  // namespace evaluation
}  // namespace reach_ros

[FILENAME: moveit_ik_solver.cpp]
/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_ros/ik/moveit_ik_solver.h>
#include <reach_ros/utils.h>

#include <boost/bind/bind.hpp>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
T clamp(const T& val, const T& low, const T& high)
{
  return std::max(low, std::min(val, high));
}

rclcpp::Node::SharedPtr getIKNode()
{
  static rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("reach_moveit_ik_solver");
  return node;
}

}  // namespace

namespace reach_ros
{
namespace ik
{
std::string MoveItIKSolver::COLLISION_OBJECT_NAME = "reach_object";

MoveItIKSolver::MoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                               double dist_threshold)
  : model_(model), jmg_(model_->getJointModelGroup(planning_group)), distance_threshold_(dist_threshold)
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group for planning group '" + planning_group + "'");
  if (!jmg_->getSolverInstance())
    throw std::runtime_error("No kinematics solver instantiated for planning group '" + planning_group +
                             "'. Check that the 'kinematics.yaml' file was loaded as a parameter");

  scene_.reset(new planning_scene::PlanningScene(model_));

  scene_pub_ = getIKNode()->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene",
                                                                               rclcpp::QoS(1).transient_local());
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_->publish(scene_msg);
}

std::vector<std::vector<double>> MoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                         const std::map<std::string, double>& seed) const
{
  moveit::core::RobotState state(model_);
  state.setToDefaultValues();

  const std::vector<std::string>& active_names = jmg_->getActiveJointModelNames();
  if (!seed.empty())
  {
    std::vector<double> seed_subset = reach::extractSubset(seed, active_names);
    if (!seed_subset.empty())
      state.setJointGroupPositions(jmg_, seed_subset);
  }
  state.update();

  const moveit::core::GroupStateValidityCallbackFn validity_callback =
      boost::bind(&MoveItIKSolver::isIKSolutionValid, this, boost::placeholders::_1, boost::placeholders::_2,
                  boost::placeholders::_3);

  const std::string tip_frame = jmg_->getSolverInstance()->getTipFrame();
  const bool found_solution = state.setFromIK(jmg_, target, tip_frame, 1, 0.0, validity_callback);
  state.update();

  if (!found_solution)
    return {};

  std::vector<double> solution;
  state.copyJointGroupPositions(jmg_, solution);

  std::vector<std::vector<double>> solutions;
  solutions.reserve(1);
  solutions.push_back(std::move(solution));
  return solutions;
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);

  return (!colliding && !too_close);
}

std::vector<std::string> MoveItIKSolver::getJointNames() const
{
  return jmg_->getActiveJointModelNames();
}

void MoveItIKSolver::addCollisionMesh(const std::string& collision_mesh_filename,
                                      const std::string& collision_mesh_frame)
{
  moveit_msgs::msg::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename, collision_mesh_frame, COLLISION_OBJECT_NAME);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  moveit_msgs::msg::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_->publish(scene_msg);
}

void MoveItIKSolver::setTouchLinks(const std::vector<std::string>& touch_links)
{
  scene_->getAllowedCollisionMatrixNonConst().setEntry(COLLISION_OBJECT_NAME, touch_links, true);
}

std::string MoveItIKSolver::getKinematicBaseFrame() const
{
  return jmg_->getSolverInstance()->getBaseFrame();
}

reach::IKSolver::ConstPtr MoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto ik_solver = std::make_shared<MoveItIKSolver>(model, planning_group, dist_threshold);

  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           ik_solver->getKinematicBaseFrame();

    ik_solver->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    ik_solver->setTouchLinks(touch_links);
  }

  return ik_solver;
}

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver(moveit::core::RobotModelConstPtr model,
                                                     const std::string& planning_group, double dist_threshold,
                                                     double dt)
  : MoveItIKSolver(model, planning_group, dist_threshold), dt_(dt)
{
}

std::vector<std::vector<double>> DiscretizedMoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                                    const std::map<std::string, double>& seed) const
{
  const static int n_discretizations = int((2.0 * M_PI) / dt_);

  std::vector<std::vector<double>> solutions;
  solutions.reserve(n_discretizations);

  for (int i = 0; i < n_discretizations; ++i)
  {
    Eigen::Isometry3d discretized_target(target * Eigen::AngleAxisd(double(i) * dt_, Eigen::Vector3d::UnitZ()));
    std::vector<std::vector<double>> tmp_sols = MoveItIKSolver::solveIK(discretized_target, seed);

    if (!tmp_sols.empty())
      solutions.push_back(tmp_sols.front());
  }

  return solutions;
}

reach::IKSolver::ConstPtr DiscretizedMoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto dt = std::abs(reach::get<double>(config, "discretization_angle"));
  double clamped_dt = clamp<double>(dt, 0.0, M_PI);
  if (std::abs(dt - clamped_dt) > 1.0e-6)
  {
    std::cout << "Clamping discretization angle between 0 and pi; new value is " << clamped_dt;
  }
  dt = clamped_dt;

  auto ik_solver = std::make_shared<DiscretizedMoveItIKSolver>(model, planning_group, dist_threshold, dt);

  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           ik_solver->getKinematicBaseFrame();

    ik_solver->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    ik_solver->setTouchLinks(touch_links);
  }

  return ik_solver;
}

}  // namespace ik
}  // namespace reach_ros

[FILENAME: transformed_point_cloud_target_pose_generator.cpp]
#include <reach_ros/target_pose_generator/transformed_point_cloud_target_pose_generator.h>
#include <reach_ros/utils.h>

#include <reach/plugin_utils.h>
#include <rclcpp/rclcpp.hpp>
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

std::vector<Eigen::Isometry3d> TransformedPointCloudTargetPoseGenerator::generate() const
{
  std::vector<Eigen::Isometry3d> poses = reach::PointCloudTargetPoseGenerator::generate();

  utils::initROS();
  auto node = rclcpp::Node::make_shared("transformed_point_cloud_target_pose_generator");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, true);

  geometry_msgs::msg::TransformStamped transform_msg;
  try
  {
    transform_msg =
        tf_buffer.lookupTransform(target_frame_, points_frame_, tf2::TimePointZero, tf2::durationFromSec(3.0));
  }
  catch (const tf2::TransformException& ex)
  {
    throw std::runtime_error(std::string("Failed to lookup transform from '") + points_frame_ + "' to '" +
                             target_frame_ + "': " + ex.what());
  }

  const Eigen::Isometry3d tf = tf2::transformToEigen(transform_msg.transform);

  for (Eigen::Isometry3d& pose : poses)
    pose = tf * pose;

  return poses;
}

}  // namespace reach_ros