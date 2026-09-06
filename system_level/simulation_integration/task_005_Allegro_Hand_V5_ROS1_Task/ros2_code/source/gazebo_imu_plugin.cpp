/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_imu_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      velocity_prev_W_(0, 0, 0)
{
}

GazeboImuPlugin::~GazeboImuPlugin() {
  updateConnection_->~Connection();
}


void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                      imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeRandomWalk",
                      imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                      imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                      imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                      imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                      imu_parameters_.accelerometer_random_walk,
                      imu_parameters_.accelerometer_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                      imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                      imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = rclcpp::Node::make_shared("gazebo_imu_plugin_" + namespace_);

  // Create ROS 2 publisher
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      "~/" + model_->GetName() + "/" + imu_topic_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  // Fill imu message.
  imu_message_.header.frame_id = frame_id_;

  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.

  // Angular velocity measurement covariance.
  imu_message_.angular_velocity_covariance[0] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_.angular_velocity_covariance[1] = 0.0;
  imu_message_.angular_velocity_covariance[2] = 0.0;
  imu_message_.angular_velocity_covariance[3] = 0.0;
  imu_message_.angular_velocity_covariance[4] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_.angular_velocity_covariance[5] = 0.0;
  imu_message_.angular_velocity_covariance[6] = 0.0;
  imu_message_.angular_velocity_covariance[7] = 0.0;
  imu_message_.angular_velocity_covariance[8] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;

  // Orientation covariance: set to -1 to indicate orientation is not provided
  imu_message_.orientation_covariance[0] = -1.0;
  for (int i = 1; i < 9; ++i) {
    imu_message_.orientation_covariance[i] = -1.0;
  }

  // Linear acceleration measurement covariance.
  imu_message_.linear_acceleration_covariance[0] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_.linear_acceleration_covariance[1] = 0.0;
  imu_message_.linear_acceleration_covariance[2] = 0.0;
  imu_message_.linear_acceleration_covariance[3] = 0.0;
  imu_message_.linear_acceleration_covariance[4] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_.linear_acceleration_covariance[5] = 0.0;
  imu_message_.linear_acceleration_covariance[6] = 0.0;
  imu_message_.linear_acceleration_covariance[7] = 0.0;
  imu_message_.linear_acceleration_covariance[8] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;

  gravity_W_ = world_->Gravity();
  imu_parameters_.gravity_magnitude = gravity_W_.Length();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
      gyroscope_bias_[i] =
          sigma_bon_g * standard_normal_distribution_(random_generator_);
      accelerometer_bias_[i] =
          sigma_bon_a * standard_normal_distribution_(random_generator_);
  }
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  assert(dt > 0.0);

  // Gyroscope noise and bias
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 *
      (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-making matrix.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] = (*angular_velocity)[i] +
        gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_);
  }

  // Accelerometer noise and bias
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
      (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-making matrix.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] +
        accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_);
  }
}

// This gets called by the world update start event.
void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
#endif

  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

#if GAZEBO_MAJOR_VERSION < 5
  ignition::math::Vector3d velocity_current_W = link_->GetWorldLinearVel();
  ignition::math::Vector3d acceleration = (velocity_current_W - velocity_prev_W_) / dt;
  ignition::math::Vector3d acceleration_I =
      C_W_I.RotateVectorReverse(acceleration - gravity_W_);

  velocity_prev_W_ = velocity_current_W;
#elif GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d acceleration_I = link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);
#else
  ignition::math::Vector3d acceleration_I = ignitionFromGazeboMath(link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_));
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();
#else
  ignition::math::Vector3d angular_vel_I = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif

  Eigen::Vector3d linear_acceleration_I(acceleration_I.X(),
                                        acceleration_I.Y(),
                                        acceleration_I.Z());
  Eigen::Vector3d angular_velocity_I(angular_vel_I.X(),
                                     angular_vel_I.Y(),
                                     angular_vel_I.Z());

  addNoise(&linear_acceleration_I, &angular_velocity_I, dt);

  // Fill IMU message using ROS 2 message struct access
  // Set timestamp using ROS 2 node clock
  rclcpp::Time now = node_->now();
  imu_message_.header.stamp = now;
  imu_message_.header.frame_id = frame_id_;

  // Set orientation
  imu_message_.orientation.x = C_W_I.X();
  imu_message_.orientation.y = C_W_I.Y();
  imu_message_.orientation.z = C_W_I.Z();
  imu_message_.orientation.w = C_W_I.W();

  // Set linear acceleration
  imu_message_.linear_acceleration.x = linear_acceleration_I[0];
  imu_message_.linear_acceleration.y = linear_acceleration_I[1];
  imu_message_.linear_acceleration.z = linear_acceleration_I[2];

  // Set angular velocity
  imu_message_.angular_velocity.x = angular_velocity_I[0];
  imu_message_.angular_velocity.y = angular_velocity_I[1];
  imu_message_.angular_velocity.z = angular_velocity_I[2];

  imu_pub_->publish(imu_message_);

  seq_++;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}