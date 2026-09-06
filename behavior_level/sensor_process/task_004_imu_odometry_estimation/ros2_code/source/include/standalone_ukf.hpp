/*
 * Standalone header that replicates the minimal robot_localization types
 * needed to test projectSigmaPoint without depending on the robot_localization package.
 */
#ifndef STANDALONE_UKF_HPP_
#define STANDALONE_UKF_HPP_

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "rclcpp/rclcpp.hpp"

namespace robot_localization
{

// State member indices (from robot_localization/filter_common.hpp)
enum StateMemberIndices
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

const int STATE_SIZE = 15;

namespace filter_utilities
{
  inline double toSec(const rclcpp::Duration & d)
  {
    return d.seconds();
  }
}

// Minimal standalone UKF class for testing projectSigmaPoint
class StandaloneUkf
{
public:
  StandaloneUkf()
  {
    state_ = Eigen::VectorXd::Zero(STATE_SIZE);
    transfer_function_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    estimate_error_covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01;
    process_noise_covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.001;

    size_t sigma_count = (STATE_SIZE << 1) + 1;
    sigma_points_.resize(sigma_count, Eigen::VectorXd::Zero(STATE_SIZE));
    state_weights_.resize(sigma_count);
    covar_weights_.resize(sigma_count);
  }

  void setConstants(double alpha, double kappa, double beta)
  {
    size_t sigma_count = (STATE_SIZE << 1) + 1;
    lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;

    state_weights_[0] = lambda_ / (STATE_SIZE + lambda_);
    covar_weights_[0] = state_weights_[0] + (1 - (alpha * alpha) + beta);
    sigma_points_[0].setZero();
    for (size_t i = 1; i < sigma_count; ++i) {
      sigma_points_[i].setZero();
      state_weights_[i] = 1.0 / (2.0 * (STATE_SIZE + lambda_));
      covar_weights_[i] = state_weights_[i];
    }
  }

  void setState(const Eigen::VectorXd & state) { state_ = state; }
  const Eigen::VectorXd & getState() const { return state_; }

  void setEstimateErrorCovariance(const Eigen::MatrixXd & cov)
  {
    estimate_error_covariance_ = cov;
  }

  void generateSigmaPoints()
  {
    weighted_covar_sqrt_ =
      ((static_cast<double>(STATE_SIZE) + lambda_) * estimate_error_covariance_).llt().matrixL();

    sigma_points_[0] = state_;
    for (size_t sigma_ind = 0; sigma_ind < static_cast<size_t>(STATE_SIZE); ++sigma_ind) {
      sigma_points_[sigma_ind + 1] = state_ + weighted_covar_sqrt_.col(sigma_ind);
      sigma_points_[sigma_ind + 1 + STATE_SIZE] = state_ - weighted_covar_sqrt_.col(sigma_ind);
    }
  }

  void projectSigmaPoint(Eigen::VectorXd & sigma_point, const rclcpp::Duration & delta)
  {
    double delta_sec = filter_utilities::toSec(delta);

    double roll = sigma_point(StateMemberRoll);
    double pitch = sigma_point(StateMemberPitch);
    double yaw = sigma_point(StateMemberYaw);

    double sr = ::sin(roll);
    double cr = ::cos(roll);
    double sp = ::sin(pitch);
    double cp = ::cos(pitch);
    double sy = ::sin(yaw);
    double cy = ::cos(yaw);
    double tp = ::tan(pitch);
    double cpi = 1.0 / cp;

    transfer_function_.setIdentity();

    // Position update from velocity (3D rotation: body to global)
    transfer_function_(StateMemberX, StateMemberVx) = cy * cp * delta_sec;
    transfer_function_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta_sec;
    transfer_function_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta_sec;

    transfer_function_(StateMemberY, StateMemberVx) = sy * cp * delta_sec;
    transfer_function_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta_sec;
    transfer_function_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta_sec;

    transfer_function_(StateMemberZ, StateMemberVx) = -sp * delta_sec;
    transfer_function_(StateMemberZ, StateMemberVy) = cp * sr * delta_sec;
    transfer_function_(StateMemberZ, StateMemberVz) = cp * cr * delta_sec;

    // Position update from acceleration
    transfer_function_(StateMemberX, StateMemberAx) = 0.5 * cy * cp * delta_sec * delta_sec;
    transfer_function_(StateMemberX, StateMemberAy) = 0.5 * (cy * sp * sr - sy * cr) * delta_sec * delta_sec;
    transfer_function_(StateMemberX, StateMemberAz) = 0.5 * (cy * sp * cr + sy * sr) * delta_sec * delta_sec;

    transfer_function_(StateMemberY, StateMemberAx) = 0.5 * sy * cp * delta_sec * delta_sec;
    transfer_function_(StateMemberY, StateMemberAy) = 0.5 * (sy * sp * sr + cy * cr) * delta_sec * delta_sec;
    transfer_function_(StateMemberY, StateMemberAz) = 0.5 * (sy * sp * cr - cy * sr) * delta_sec * delta_sec;

    transfer_function_(StateMemberZ, StateMemberAx) = 0.5 * (-sp) * delta_sec * delta_sec;
    transfer_function_(StateMemberZ, StateMemberAy) = 0.5 * cp * sr * delta_sec * delta_sec;
    transfer_function_(StateMemberZ, StateMemberAz) = 0.5 * cp * cr * delta_sec * delta_sec;

    // Orientation update from angular velocity
    transfer_function_(StateMemberRoll, StateMemberVroll) = delta_sec;
    transfer_function_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_sec;
    transfer_function_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_sec;

    transfer_function_(StateMemberPitch, StateMemberVpitch) = cr * delta_sec;
    transfer_function_(StateMemberPitch, StateMemberVyaw) = -sr * delta_sec;

    transfer_function_(StateMemberYaw, StateMemberVroll) = 0.0;
    transfer_function_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_sec;
    transfer_function_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_sec;

    // Velocity update from acceleration
    transfer_function_(StateMemberVx, StateMemberAx) = delta_sec;
    transfer_function_(StateMemberVy, StateMemberAy) = delta_sec;
    transfer_function_(StateMemberVz, StateMemberAz) = delta_sec;

    sigma_point.applyOnTheLeft(transfer_function_);
  }

  void predict(const rclcpp::Time & /*reference_time*/, const rclcpp::Duration & delta)
  {
    generateSigmaPoints();

    double roll_sum_x = 0.0, roll_sum_y = 0.0;
    double pitch_sum_x = 0.0, pitch_sum_y = 0.0;
    double yaw_sum_x = 0.0, yaw_sum_y = 0.0;

    state_.setZero();
    for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
      projectSigmaPoint(sigma_points_[sigma_ind], delta);
      state_.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];

      roll_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberRoll));
      roll_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberRoll));
      pitch_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberPitch));
      pitch_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberPitch));
      yaw_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberYaw));
      yaw_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberYaw));
    }

    state_(StateMemberRoll) = ::atan2(roll_sum_y, roll_sum_x);
    state_(StateMemberPitch) = ::atan2(pitch_sum_y, pitch_sum_x);
    state_(StateMemberYaw) = ::atan2(yaw_sum_y, yaw_sum_x);

    estimate_error_covariance_.setZero();
    Eigen::VectorXd sigma_diff(STATE_SIZE);
    for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
      sigma_diff = sigma_points_[sigma_ind] - state_;
      auto normalize = [](double a) -> double {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
      };
      sigma_diff(StateMemberRoll) = normalize(sigma_diff(StateMemberRoll));
      sigma_diff(StateMemberPitch) = normalize(sigma_diff(StateMemberPitch));
      sigma_diff(StateMemberYaw) = normalize(sigma_diff(StateMemberYaw));

      estimate_error_covariance_.noalias() += covar_weights_[sigma_ind] *
        (sigma_diff * sigma_diff.transpose());
    }

    estimate_error_covariance_.noalias() += delta.seconds() * process_noise_covariance_;

    state_(StateMemberRoll) = ::atan2(::sin(state_(StateMemberRoll)), ::cos(state_(StateMemberRoll)));
    state_(StateMemberPitch) = ::atan2(::sin(state_(StateMemberPitch)), ::cos(state_(StateMemberPitch)));
    state_(StateMemberYaw) = ::atan2(::sin(state_(StateMemberYaw)), ::cos(state_(StateMemberYaw)));
  }

private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd transfer_function_;
  Eigen::MatrixXd estimate_error_covariance_;
  Eigen::MatrixXd process_noise_covariance_;
  Eigen::MatrixXd weighted_covar_sqrt_;
  double lambda_{0.0};
  std::vector<Eigen::VectorXd> sigma_points_;
  std::vector<double> state_weights_;
  std::vector<double> covar_weights_;
};

}  // namespace robot_localization

#endif  // STANDALONE_UKF_HPP_