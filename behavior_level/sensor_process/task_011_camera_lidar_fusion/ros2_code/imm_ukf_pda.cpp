bool ImmUkfPda::updateNecessaryTransform()
{
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped;

    // lookup tracking_frame_ <- input_header_.frame_id
    transform_stamped = tf_buffer_->lookupTransform(
      tracking_frame_, input_header_.frame_id, tf2::TimePointZero);
    tf2::fromMsg(transform_stamped.transform, local2global_);

    // lookup vectormap_frame_ <- tracking_frame_
    transform_stamped = tf_buffer_->lookupTransform(
      vectormap_frame_, tracking_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform_stamped.transform, tracking_frame2vectormap_frame_);

    // lookup tracking_frame_ <- vectormap_frame_
    transform_stamped = tf_buffer_->lookupTransform(
      tracking_frame_, vectormap_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform_stamped.transform, vectormap_frame2tracking_frame_);

    // lookup tracking_frame_ <- lane_frame_
    transform_stamped = tf_buffer_->lookupTransform(
      tracking_frame_, vectormap_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform_stamped.transform, lane_frame2tracking_frame_);

    // lookup lane_frame_ <- tracking_frame_
    transform_stamped = tf_buffer_->lookupTransform(
      vectormap_frame_, tracking_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform_stamped.transform, tracking_frame2lane_frame_);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(rclcpp::get_logger("ImmUkfPda"), "Could not find coordinate transformation: %s", ex.what());
    return false;
  }
  return true;
}

void ImmUkfPda::tracker(const autoware_msgs::DetectedObjectArray& input,
                        autoware_msgs::DetectedObjectArray& detected_objects_output)
{
  rclcpp::Time current_time(input.header.stamp);
  double timestamp = current_time.seconds();
  double dt = timestamp_ == 0.0 ? 0.0 : timestamp - timestamp_;
  timestamp_ = timestamp;

  std::vector<bool> matching_vec(input.objects.size(), false);
  std::vector<autoware_msgs::DetectedObject> object_vec;

  if (!init_)
  {
    initTracker(input, timestamp);
    return;
  }

  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ == TrackingState::Die)
      continue;

    targets_[i].prediction(dt);

    bool success = probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);

    if (success)
    {
      targets_[i].update(object_vec);
    }
  }

  makeNewTargets(timestamp, input, matching_vec);

  staticClassification();

  makeOutput(input, matching_vec, detected_objects_output);

  removeUnnecessaryTarget();
}