void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (pause_scan_processing_)
  {
    RCLCPP_INFO(this->get_logger(), "Mapping paused, ignoring scan");
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_base_tf;
  geometry_msgs::msg::TransformStamped odom_to_base;

  try
  {
    map_to_base_tf = tf_buffer_->lookupTransform(p_map_frame_, p_base_frame_, tf2::TimePoint(tf2_ros::fromMsg(scan.header.stamp)));
    odom_to_base = tf_buffer_->lookupTransform(p_odom_frame_, p_base_frame_, tf2::TimePoint(tf2_ros::fromMsg(scan.header.stamp)));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    return;
  }

  // Compute map_to_odom_ = map_to_base_tf * odom_to_base.inverse()
  tf2::Transform tf_map_to_base, tf_odom_to_base;
  tf2::fromMsg(map_to_base_tf.transform, tf_map_to_base);
  tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);
  tf2::Transform tf_odom_to_base_inv = tf_odom_to_base.inverse();
  tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base_inv;
  map_to_odom_ = tf_map_to_odom;

  hectorslam::DataContainer laserScanContainer;
  rosLaserScanToDataContainer(scan, laserScanContainer, 1.0f);

  if (!initial_pose_set_)
  {
    RCLCPP_INFO(this->get_logger(), "Initial pose not set, skipping scan processing");
    return;
  }

  if (!slamProcessor->update(laserScanContainer, initial_pose_))
  {
    RCLCPP_WARN(this->get_logger(), "SLAM update failed");
    return;
  }

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = scan.header;
  pose_msg.header.frame_id = p_map_frame_;
  pose_msg.pose.position.x = slamProcessor->getLastPose().x();
  pose_msg.pose.position.y = slamProcessor->getLastPose().y();
  pose_msg.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, slamProcessor->getLastPose().z());
  pose_msg.pose.orientation = tf2::toMsg(q);

  // Publish pose with transient_local QoS
  rclcpp::QoS transient_local_qos(rclcpp::KeepLast(1));
  transient_local_qos.transient_local();

  posePublisher_.publish(std::move(pose_msg));

  if (p_pub_odometry_)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = scan.header;
    odom_msg.header.frame_id = p_odom_frame_;
    odom_msg.child_frame_id = p_base_frame_;
    odom_msg.pose.pose = pose_msg.pose;
    odometryPublisher_.publish(std::move(odom_msg));
  }
}