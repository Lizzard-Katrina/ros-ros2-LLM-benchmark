namespace robot_localization
{
void Ukf::projectSigmaPoint(Eigen::VectorXd & sigma_point, const rclcpp::Duration & delta)
{
  // Extract current roll, pitch, yaw
  double roll = sigma_point(StateMemberRoll);
  double pitch = sigma_point(StateMemberPitch);
  double yaw = sigma_point(StateMemberYaw);

  // Compute rotation matrix from body frame to global frame using RZYX (yaw-pitch-roll)
  double cr = std::cos(roll);
  double sr = std::sin(roll);
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);

  // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::Matrix3d R;
  R(0, 0) = cy * cp;
  R(0, 1) = cy * sp * sr - sy * cr;
  R(0, 2) = cy * sp * cr + sy * sr;
  R(1, 0) = sy * cp;
  R(1, 1) = sy * sp * sr + cy * cr;
  R(1, 2) = sy * sp * cr - cy * sr;
  R(2, 0) = -sp;
  R(2, 1) = cp * sr;
  R(2, 2) = cp * cr;

  // Extract body-frame linear velocities and accelerations
  Eigen::Vector3d body_vel(sigma_point(StateMemberVx), sigma_point(StateMemberVy), sigma_point(StateMemberVz));
  Eigen::Vector3d body_accel(sigma_point(StateMemberAx), sigma_point(StateMemberAy), sigma_point(StateMemberAz));

  // Map body-frame linear velocity to global frame
  Eigen::Vector3d global_vel = R * body_vel;

  // Map body-frame linear acceleration to global frame
  Eigen::Vector3d global_accel = R * body_accel;

  // Update position: x = x + v * dt + 0.5 * a * dt^2
  sigma_point(StateMemberX) += global_vel(0) * delta.seconds() + 0.5 * global_accel(0) * delta.seconds() * delta.seconds();
  sigma_point(StateMemberY) += global_vel(1) * delta.seconds() + 0.5 * global_accel(1) * delta.seconds() * delta.seconds();
  sigma_point(StateMemberZ) += global_vel(2) * delta.seconds() + 0.5 * global_accel(2) * delta.seconds() * delta.seconds();

  // Update velocity: v = v + a * dt
  sigma_point(StateMemberVx) += global_accel(0) * delta.seconds();
  sigma_point(StateMemberVy) += global_accel(1) * delta.seconds();
  sigma_point(StateMemberVz) += global_accel(2) * delta.seconds();

  // Extract angular velocities in body frame
  double p = sigma_point(StateMemberP);
  double q = sigma_point(StateMemberQ);
  double r = sigma_point(StateMemberR);

  // Compute Euler angle derivatives from body angular rates
  // Using the standard mapping:
  // [roll_dot; pitch_dot; yaw_dot] = T * [p; q; r]
  // where
  // T = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch);
  //      0, cos(roll),          -sin(roll);
  //      0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]

  double tan_pitch = std::tan(pitch);
  double cos_pitch = std::cos(pitch);

  Eigen::Matrix3d T;
  T << 1.0, sr * tan_pitch, cr * tan_pitch,
       0.0, cr,           -sr,
       0.0, sr / cos_pitch, cr / cos_pitch;

  Eigen::Vector3d body_rates(p, q, r);
  Eigen::Vector3d euler_dot = T * body_rates;

  // Update Euler angles: angle = angle + angle_dot * dt
  sigma_point(StateMemberRoll) += euler_dot(0) * delta.seconds();
  sigma_point(StateMemberPitch) += euler_dot(1) * delta.seconds();
  sigma_point(StateMemberYaw) += euler_dot(2) * delta.seconds();

  // Normalize angles to [-pi, pi]
  sigma_point(StateMemberRoll) = angles::normalize_angle(sigma_point(StateMemberRoll));
  sigma_point(StateMemberPitch) = angles::normalize_angle(sigma_point(StateMemberPitch));
  sigma_point(StateMemberYaw) = angles::normalize_angle(sigma_point(StateMemberYaw));

  // Apply transfer function matrix on the left to sigma_point to update it in-place
  sigma_point = transfer_function_ * sigma_point;
}

}  // namespace robot_localization