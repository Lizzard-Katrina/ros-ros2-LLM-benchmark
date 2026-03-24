void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg)
{
  rclcpp::Time trigger_stamp_local;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trigger_stamp_local = trigger_stamp;
  }

  if (!run_without_trigger && trigger_stamp_local == rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    RCLCPP_WARN(this->get_logger(), "%s No radar trigger received yet, skipping radar scan", kPrefix.c_str());
    return;
  }

  if (run_without_trigger)
  {
    trigger_stamp_local = radar_scan_msg->header.stamp;
  }

  processRadarData(*radar_scan_msg, trigger_stamp_local);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    trigger_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
}