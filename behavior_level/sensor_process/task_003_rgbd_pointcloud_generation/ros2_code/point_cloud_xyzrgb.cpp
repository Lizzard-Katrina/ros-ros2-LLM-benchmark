namespace depth_image_proc
{

void PointCloudXyzrgbNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,  // 10 seconds
      "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // 1. Resolution Scaling
  if (depth_msg->width != rgb_msg_in->width || depth_msg->height != rgb_msg_in->height) {
    float scale_x = static_cast<float>(depth_msg->width) / static_cast<float>(rgb_msg_in->width);
    float scale_y = static_cast<float>(depth_msg->height) / static_cast<float>(rgb_msg_in->height);

    // Scale intrinsics manually
    model_.fx() *= scale_x;
    model_.fy() *= scale_y;
    model_.cx() *= scale_x;
    model_.cy() *= scale_y;
  }

  // 2. Encoding Logic: Determine color offsets
  int red_offset = -1, green_offset = -1, blue_offset = -1;
  if (rgb_msg_in->encoding == sensor_msgs::image_encodings::RGB8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
  } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::BGR8) {
    blue_offset = 0;
    green_offset = 1;
    red_offset = 2;
  } else if (rgb_msg_in->encoding == sensor_msgs::image_encodings::MONO8) {
    red_offset = 0;
    green_offset = 0;
    blue_offset = 0;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Unsupported RGB encoding: %s", rgb_msg_in->encoding.c_str());
    return;
  }

  // 3. Initialize PointCloud2 message
  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(cloud_msg->height * cloud_msg->width);

  // 4. Kernel Execution: Project depth and color to point cloud
  // Convert depth image to point cloud xyz
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepth<uint16_t>(depth_msg, model_, invalid_depth_, cloud_msg.get());
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepth<float>(depth_msg, model_, invalid_depth_, cloud_msg.get());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
    return;
  }

  // Convert RGB image to point cloud rgb field
  convertRgb(rgb_msg_in, red_offset, green_offset, blue_offset, cloud_msg.get());

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc