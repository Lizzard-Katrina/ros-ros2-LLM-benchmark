namespace depth_image_proc
{

void PointCloudXyzrgbRadialNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN(
      get_logger(), "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // Scale intrinsics if resolutions differ
  double ratio_x = static_cast<double>(rgb_msg_in->width) / static_cast<double>(depth_msg->width);
  double ratio_y = static_cast<double>(rgb_msg_in->height) / static_cast<double>(depth_msg->height);
  if (ratio_x != 1.0 || ratio_y != 1.0) {
    model_.fx() *= ratio_x;
    model_.cx() *= ratio_x;
    model_.fy() *= ratio_y;
    model_.cy() *= ratio_y;
  }

  // Update transform_ only if CameraInfo or image dimensions changed
  if (info_msg->header.frame_id != last_frame_id_ ||
      depth_msg->width != last_width_ || depth_msg->height != last_height_) {
    transform_.initMatrix(model_);
    last_frame_id_ = info_msg->header.frame_id;
    last_width_ = depth_msg->width;
    last_height_ = depth_msg->height;
  }

  // Prepare point cloud message
  std::unique_ptr<PointCloud2> cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(cloud_msg->height * cloud_msg->width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

  // Color mapping offsets and step
  const int red_offset = 2;
  const int green_offset = 1;
  const int blue_offset = 0;
  const int color_step = rgb_msg_in->step;

  // Convert depth and RGB to point cloud
  cv::Mat depth_image = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  cv::Mat rgb_image = cv_bridge::toCvShare(rgb_msg_in, sensor_msgs::image_encodings::RGB8)->image;

  for (int v = 0; v < static_cast<int>(depth_msg->height); ++v) {
    const float* depth_row = depth_image.ptr<float>(v);
    const uint8_t* rgb_row = rgb_image.ptr<uint8_t>(static_cast<int>(v * ratio_y));
    for (int u = 0; u < static_cast<int>(depth_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
      float depth = depth_row[u];
      if (std::isnan(depth) || depth <= 0.001f) {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_r = *iter_g = *iter_b = 0;
        continue;
      }
      // Compute 3D point
      float x = (u - model_.cx()) * depth / model_.fx();
      float y = (v - model_.cy()) * depth / model_.fy();
      float z = depth;

      // Apply radial transform
      Eigen::Vector3f pt(x, y, z);
      pt = transform_ * pt;

      *iter_x = pt.x();
      *iter_y = pt.y();
      *iter_z = pt.z();

      // Map color
      int rgb_u = static_cast<int>(u * ratio_x);
      int rgb_v = static_cast<int>(v * ratio_y);
      const uint8_t* pixel_ptr = &rgb_image.ptr<uint8_t>(rgb_v)[rgb_u * 3];
      *iter_r = pixel_ptr[red_offset];
      *iter_g = pixel_ptr[green_offset];
      *iter_b = pixel_ptr[blue_offset];
    }
  }

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc