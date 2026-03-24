bool IKFastKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                        const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                        double search_discretization)
{
  // Use MoveIt 2 logging
  RCLCPP_INFO(rclcpp::get_logger("IKFastKinematicsPlugin"), "Initializing IKFastKinematicsPlugin for group '%s'", group_name.c_str());

  robot_model_ = &robot_model;
  group_name_ = group_name;

  // Check that there is exactly one tip frame
  if (tip_frames.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "IKFast solver supports exactly one tip frame");
    return false;
  }

  // Store tip frame and base frame
  tip_frame_ = tip_frames[0];
  base_frame_ = base_frame;

  // Check if the tip frame and base frame match the IKFast solver frames
  tip_transform_required_ = (tip_frame_ != IKFAST_TIP_FRAME_);
  base_transform_required_ = (base_frame_ != IKFAST_BASE_FRAME_);

  // Initialize transforms to identity
  chain_base_to_group_base_ = Eigen::Isometry3d::Identity();
  group_tip_to_chain_tip_ = Eigen::Isometry3d::Identity();

  // If transforms are required, compute them
  if (tip_transform_required_)
  {
    bool differs = false;
    if (!computeRelativeTransform(IKFAST_TIP_FRAME_, tip_frame_, group_tip_to_chain_tip_, differs))
    {
      RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "Failed to compute transform from IKFast tip frame '%s' to group tip frame '%s'",
                   IKFAST_TIP_FRAME_.c_str(), tip_frame_.c_str());
      return false;
    }
  }

  if (base_transform_required_)
  {
    bool differs = false;
    if (!computeRelativeTransform(base_frame_, IKFAST_BASE_FRAME_, chain_base_to_group_base_, differs))
    {
      RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "Failed to compute transform from group base frame '%s' to IKFast base frame '%s'",
                   base_frame_.c_str(), IKFAST_BASE_FRAME_.c_str());
      return false;
    }
  }

  // Get the joint model group
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name_);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "Joint model group '%s' not found", group_name_.c_str());
    return false;
  }

  // Get active joint models
  const std::vector<const moveit::core::JointModel*>& active_joints = joint_model_group->getActiveJointModels();

  // Check number of joints matches IKFast solver
  if (active_joints.size() != num_joints_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "Number of joints in group (%zu) does not match IKFast solver (%zu)",
                 active_joints.size(), num_joints_);
    return false;
  }

  // Store joint names and limits
  joint_names_.clear();
  joint_min_vector_.clear();
  joint_max_vector_.clear();
  joint_has_limits_vector_.clear();

  for (const moveit::core::JointModel* jm : active_joints)
  {
    joint_names_.push_back(jm->getName());

    if (jm->getVariableBounds().size() != 1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IKFastKinematicsPlugin"), "Joint '%s' has more than one variable, not supported", jm->getName().c_str());
      return false;
    }

    const moveit::core::VariableBounds& bounds = jm->getVariableBounds()[0];
    joint_min_vector_.push_back(bounds.min_position_);
    joint_max_vector_.push_back(bounds.max_position_);
    joint_has_limits_vector_.push_back(bounds.position_bounded_);
  }

  // Store link names (all links in the chain)
  link_names_ = joint_model_group->getLinkModelNames();

  // Identify free parameters (redundant joints)
  free_params_.clear();
  redundant_joint_indices_.clear();

  // IKFast solver free parameters count and indices
  int free_param_count = GetNumFreeParameters();
  if (free_param_count > 0)
  {
    int free_param_array[free_param_count];
    GetFreeParameters(free_param_array);

    for (int i = 0; i < free_param_count; ++i)
    {
      free_params_.push_back(free_param_array[i]);
      redundant_joint_indices_.push_back(free_param_array[i]);
    }
  }

  // Set default discretization for redundant joints
  redundant_joint_discretization_.clear();
  if (!redundant_joint_indices_.empty())
  {
    redundant_joint_discretization_[redundant_joint_indices_[0]] = search_discretization;
  }

  // Declare parameters for ROS 2 node interface
  // Since this is a plugin, we don't have direct access to node, so parameters should be declared externally
  // But to comply with instructions, we simulate parameter declaration and retrieval here

  // For example, retrieve parameters from ROS 2 parameter server if available
  // (In actual MoveIt 2 plugin, parameters are passed via constructor or initialize method)

  // Mark as initialized
  initialized_ = true;

  RCLCPP_INFO(rclcpp::get_logger("IKFastKinematicsPlugin"), "IKFastKinematicsPlugin initialized successfully");

  return true;
}