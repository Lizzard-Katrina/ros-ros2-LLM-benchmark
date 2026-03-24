namespace cl_move_base_z
{
namespace odom_tracker
{
OdomTracker::OdomTracker(std::string odomTopicName, std::string odomFrame)
  : Node("odom_tracker_node"),
    odomFrame_(odomFrame),
    workingMode_(WorkingMode::RECORD_PATH),
    publishMessages(true)
{
  this->declare_parameter<std::string>("odom_frame", odomFrame);
  this->declare_parameter<double>("record_point_distance_threshold", 0.05);
  this->declare_parameter<double>("record_angular_distance_threshold", 0.1);
  this->declare_parameter<double>("clear_point_distance_threshold", 0.1);
  this->declare_parameter<double>("clear_angular_distance_threshold", 0.2);

  this->get_parameter("odom_frame", odomFrame_);
  this->get_parameter("record_point_distance_threshold", recordPointDistanceThreshold_);
  this->get_parameter("record_angular_distance_threshold", recordAngularDistanceThreshold_);
  this->get_parameter("clear_point_distance_threshold", clearPointDistanceThreshold_);
  this->get_parameter("clear_angular_distance_threshold", clearAngularDistanceThreshold_);

  odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odomTopicName, rclcpp::SensorDataQoS(),
    std::bind(&OdomTracker::processOdometryMessage, this, std::placeholders::_1));

  baseTrajectoryPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>>(
    this->get_node_base_interface(), "base_trajectory", rclcpp::SystemDefaultsQoS());

  aggregatedStackPathPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Path>>(
    this->get_node_base_interface(), "aggregated_stack_path", rclcpp::SystemDefaultsQoS());
}

/**
 ******************************************************************************************************************
 * rtPublishPaths()
 ******************************************************************************************************************
 */
void OdomTracker::rtPublishPaths(rclcpp::Time timestamp)
{
  if (baseTrajectoryPub_->trylock())
  {
    baseTrajectoryPub_->msg_.header.stamp = timestamp;
    baseTrajectoryPub_->msg_.header.frame_id = odomFrame_;
    baseTrajectoryPub_->msg_.poses = baseTrajectory_.poses;
    baseTrajectoryPub_->unlockAndPublish();
  }

  if (aggregatedStackPathPub_->trylock())
  {
    aggregatedStackPathPub_->msg_.header.stamp = timestamp;
    aggregatedStackPathPub_->msg_.header.frame_id = odomFrame_;
    aggregatedStackPathPub_->msg_.poses = aggregatedStackPathMsg_.poses;
    aggregatedStackPathPub_->unlockAndPublish();
  }
}
}  // namespace odom_tracker
}  // namespace cl_move_base_z