namespace turtlesim
{

bool Turtle::update(
  double dt, QPainter & path_painter, const QImage & path_image,
  qreal canvas_width, qreal canvas_height)
{
  bool modified = false;
  qreal old_orient = orient_;
  QPointF old_pos = pos_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it) {
    const TeleportRequest & req = *it;

    if (req.relative) {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += -std::sin(orient_) * req.linear;
    } else {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // Process any action requests
  if (rotate_absolute_goal_handle_) {
    // Check if there was a cancel request
    if (rotate_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "Rotation goal canceled");
      rotate_absolute_goal_handle_->canceled(rotate_absolute_result_);
      rotate_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } else {
      double theta = normalizeAngle(rotate_absolute_goal_handle_->get_goal()->theta);
      double remaining = normalizeAngle(theta - static_cast<float>(orient_));

      // Update result
      rotate_absolute_result_->delta =
        normalizeAngle(static_cast<float>(rotate_absolute_start_orient_ - orient_));

      // Update feedback
      rotate_absolute_feedback_->remaining = remaining;
      rotate_absolute_goal_handle_->publish_feedback(rotate_absolute_feedback_);

      // Check stopping condition
      if (fabs(normalizeAngle(static_cast<float>(orient_) - theta)) < 0.02) {
        RCLCPP_INFO(nh_->get_logger(), "Rotation goal completed successfully");
        rotate_absolute_goal_handle_->succeed(rotate_absolute_result_);
        rotate_absolute_goal_handle_ = nullptr;
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      } else {
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = remaining < 0.0 ? -1.0 : 1.0;
        last_command_time_ = nh_->now();
      }
    }
  }

  // 1. MOTION: Update the turtle's 'orient_' and 'pos_' based on 'lin_vel_x_', 
  //    'lin_vel_y_', and 'ang_vel_' over 'dt'. Support both holonomic and 
  //    non-holonomic movements. Ensure 'orient_' remains normalized.
  bool holonomic = false;
  nh_->get_parameter_or("holonomic", holonomic, false);

  if (holonomic) {
    pos_.rx() += lin_vel_x_ * dt;
    pos_.ry() += lin_vel_y_ * dt;
  } else {
    pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt;
    pos_.ry() += -std::sin(orient_) * lin_vel_x_ * dt;
  }
  orient_ += ang_vel_ * dt;
  orient_ = normalizeAngle(orient_);

  // 2. BOUNDARY SAFETY: Implement wall-collision logic. The turtle is 
  //    constrained within [0, canvas_width] and [0, canvas_height]. 
  //    Maintain the original requirement to log a warning upon collision.
  bool collided = false;
  if (pos_.x() < 0.0) {
    pos_.setX(0.0);
    collided = true;
  }
  if (pos_.x() > canvas_width) {
    pos_.setX(canvas_width);
    collided = true;
  }
  if (pos_.y() < 0.0) {
    pos_.setY(0.0);
    collided = true;
  }
  if (pos_.y() > canvas_height) {
    pos_.setY(canvas_height);
    collided = true;
  }
  if (collided) {
    RCLCPP_WARN(nh_->get_logger(), "Turtle hit the wall and was constrained within boundaries");
  }

  // 3. SONAR SENSING: Derive and implement a virtual sonar. 
  //    - It must sample a 30-degree Field of View (FOV) centered at 'orient_'.
  //    - Calculate the analytical intersection distance to the four window 
  //      boundaries for rays within this FOV.
  //    - Identify and store the 'shortest' distance (First Echo) to the nearest wall.
  const double sonar_fov = 30.0 * M_PI / 180.0;  // 30 degrees in radians
  const int num_rays = 31;  // sample rays within FOV
  double min_distance = std::numeric_limits<double>::max();

  for (int i = 0; i < num_rays; ++i) {
    double ray_angle = orient_ - sonar_fov / 2.0 + i * (sonar_fov / (num_rays - 1));
    ray_angle = normalizeAngle(ray_angle);

    // Ray direction vector
    double dx = std::cos(ray_angle);
    double dy = -std::sin(ray_angle);  // Qt y-axis is top-down

    // Calculate intersection distances to each boundary
    // Left boundary (x=0)
    double dist_left = std::numeric_limits<double>::max();
    if (dx < 0.0) {
      dist_left = (0.0 - pos_.x()) / dx;
    }
    // Right boundary (x=canvas_width)
    double dist_right = std::numeric_limits<double>::max();
    if (dx > 0.0) {
      dist_right = (canvas_width - pos_.x()) / dx;
    }
    // Top boundary (y=0)
    double dist_top = std::numeric_limits<double>::max();
    if (dy < 0.0) {
      dist_top = (0.0 - pos_.y()) / dy;
    }
    // Bottom boundary (y=canvas_height)
    double dist_bottom = std::numeric_limits<double>::max();
    if (dy > 0.0) {
      dist_bottom = (canvas_height - pos_.y()) / dy;
    }

    // Find the minimal positive intersection distance
    double dist = std::min({dist_left, dist_right, dist_top, dist_bottom});
    if (dist > 0.0 && dist < min_distance) {
      min_distance = dist;
    }
  }

  sonar_distance_ = min_distance;

  // 4. COORDINATE MAPPING: Respect the turtlesim convention where the internal 
  //    'pos_.y()' is top-down (Qt frame), but the 'Pose' message expects 
  //    bottom-up coordinates relative to 'canvas_height'.
  // (Handled below in pose message publishing)

  // Publish pose of the turtle
  auto p = std::make_unique<turtlesim_msgs::msg::Pose>();
  p->x = pos_.x();
  p->y = canvas_height - pos_.y();
  p->theta = orient_;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  // Figure out (and publish) the color underneath the turtle
  {
    auto color = std::make_unique<turtlesim_msgs::msg::Color>();
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color->r = qRed(pixel);
    color->g = qGreen(pixel);
    color->b = qBlue(pixel);
    color_pub_->publish(std::move(color));
  }

  RCLCPP_DEBUG(
    nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f",
    nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient) {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos) {
    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

}  // namespace turtlesim