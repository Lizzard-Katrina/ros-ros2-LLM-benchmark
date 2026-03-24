namespace darknet_ros {

void YoloObjectDetector::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    {
      std::lock_guard<std::mutex> lock(mutexImageCallback_);
      camImageCopy_ = cv_ptr->image.clone();
      imageHeader_ = msg->header;
      imageStatus_ = true;
    }
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void* YoloObjectDetector::publishInThread() {
  std::lock_guard<std::mutex> lock(mutexImageCallback_);

  darknet_ros_msgs::msg::BoundingBoxes boundingBoxesMsg;
  boundingBoxesMsg.header = imageHeader_;

  boundingBoxesMsg.bounding_boxes.reserve(roiBoxes_[0].num);

  for (int i = 0; i < roiBoxes_[0].num; ++i) {
    const RosBox_& box = roiBoxes_[i];
    darknet_ros_msgs::msg::BoundingBox bbox;

    bbox.Class = box.Class;
    bbox.probability = box.prob;
    bbox.xmin = static_cast<int>((box.x - box.w / 2.0) * frameWidth_);
    bbox.ymin = static_cast<int>((box.y - box.h / 2.0) * frameHeight_);
    bbox.xmax = static_cast<int>((box.x + box.w / 2.0) * frameWidth_);
    bbox.ymax = static_cast<int>((box.y + box.h / 2.0) * frameHeight_);

    // Clamp to image boundaries
    if (bbox.xmin < 0) bbox.xmin = 0;
    if (bbox.ymin < 0) bbox.ymin = 0;
    if (bbox.xmax > frameWidth_) bbox.xmax = frameWidth_;
    if (bbox.ymax > frameHeight_) bbox.ymax = frameHeight_;

    boundingBoxesMsg.bounding_boxes.push_back(std::move(bbox));
  }

  boundingBoxesPublisher_->publish(std::move(boundingBoxesMsg));
  RCLCPP_INFO(this->get_logger(), "Published %zu bounding boxes.", boundingBoxesMsg.bounding_boxes.size());

  return nullptr;
}

} /* namespace darknet_ros */