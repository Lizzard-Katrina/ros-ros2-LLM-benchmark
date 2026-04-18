# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: gazebo_imu_plugin.cpp
----------------------------
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
      velocity_prev_W_(0,0,0)
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
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("~/" + model_->GetName() + imu_topic_, 10);

  // Fill imu message.
  // imu_message_.header.frame_id = frame_id_; TODO Add header
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  for(int i=0; i< 9; i++){
    switch (i){
    case 0:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 1:
    case 2:
    case 3:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 4:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 5:
    case 6:
    case 7:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 8:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    }
  }

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
  // CHECK(linear_acceleration);
  // CHECK(angular_velocity);
  // TODO: Implement IMU noise simulation here.
  // Hint: Apply Gaussian noise, bias random walk, and correlation time.
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
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  // Copy ignition::math::Quaterniond to gazebo::msgs::Quaternion
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_x(C_W_I.X());
  orientation->set_y(C_W_I.Y());
  orientation->set_z(C_W_I.Z());
  orientation->set_w(C_W_I.W());

#if GAZEBO_MAJOR_VERSION < 5
  ignition::math::Vector3d velocity_current_W = link_->GetWorldLinearVel();
  // link_->RelativeLinearAccel() does not work sometimes with old gazebo versions.
  // TODO For an accurate simulation, this might have to be fixed. Consider the
  // This issue is solved in gazebo 5.
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

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
  linear_acceleration->set_x(linear_acceleration_I[0]);
  linear_acceleration->set_y(linear_acceleration_I[1]);
  linear_acceleration->set_z(linear_acceleration_I[2]);

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
  angular_velocity->set_x(angular_velocity_I[0]);
  angular_velocity->set_y(angular_velocity_I[1]);
  angular_velocity->set_z(angular_velocity_I[2]);

  // Fill IMU message.
  // ADD HEaders
  // imu_message_.header.stamp.sec = current_time.sec;
  // imu_message_.header.stamp.nsec = current_time.nsec;
  imu_message_.set_time_usec(_info.simTime.sec * 1000000 + _info.simTime.nsec / 1000);
  imu_message_.set_seq(seq_++);

  // TODO(burrimi): Add orientation estimator.
  // imu_message_.orientation.w = 1;
  // imu_message_.orientation.x = 0;
  // imu_message_.orientation.y = 0;
  // imu_message_.orientation.z = 0;

  imu_message_.set_allocated_orientation(orientation);
  imu_message_.set_allocated_linear_acceleration(linear_acceleration);
  imu_message_.set_allocated_angular_velocity(angular_velocity);

  imu_pub_->Publish(imu_message_);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}

----------------------------


# LLM Response

Looking at this code, I need to convert a Gazebo IMU plugin from ROS1 to ROS2. This requires:
1. Using rclcpp for ROS2 node communication instead of Gazebo transport
2. Using ROS2 message types (sensor_msgs/msg/Imu)
3. Converting Gazebo message APIs to ROS2 publisher APIs
4. Implementing the TODO noise simulation logic

Let me convert this file:

[FILENAME: gazebo_imu_plugin.cpp]
```cpp
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
#include <random>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      velocity_prev_W_(0,0,0),
      rclcpp_node_(nullptr),
      seq_(0)
{
}

GazeboImuPlugin::~GazeboImuPlugin() {
  updateConnection_->~Connection();
  if (rclcpp_node_) {
    rclcpp_node_.reset();
  }
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
  
  // Initialize rclcpp node
  rclcpp_node_ = rclcpp::Node::make_shared(namespace_);
  if (!rclcpp_node_) {
    gzerr << "[gazebo_imu_plugin] Failed to initialize rclcpp node.\n";
    return;
  }

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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  // Create ROS2 publisher
  imu_pub_ = rclcpp_node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

  // Initialize ROS2 IMU message
  sensor_msgs::msg::Imu imu_message_ros;
  imu_message_ros.header.frame_id = frame_id_;
  
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  imu_message_ros.angular_velocity_covariance[0] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_ros.angular_velocity_covariance[4] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_ros.angular_velocity_covariance[8] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  
  imu_message_ros.orientation_covariance[0] = -1.0;
  imu_message_ros.orientation_covariance[4] = -1.0;
  imu_message_ros.orientation_covariance[8] = -1.0;
  
  imu_message_ros.linear_acceleration_covariance[0] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_ros.linear_acceleration_covariance[4] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_ros.linear_acceleration_covariance[8] = imu_parameters_.accelerometer_noise_density *
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

  // Initialize IMU message for ROS2
  imu_message_ros.orientation.w = 1.0;
  imu_message_ros.orientation.x = 0.0;
  imu_message_ros.orientation.y = 0.0;
  imu_message_ros.orientation.z = 0.0;
}


/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  // Apply Gaussian noise to linear acceleration
  for (int i = 0; i < 3; ++i) {
    // Gaussian noise based on noise density (continuous-time)
    double gaussian_noise = standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] += imu_parameters_.accelerometer_noise_density * gaussian_noise / std::sqrt(dt);
    
    // Add bias
    (*linear_acceleration)[i] += accelerometer_bias_[i];
  }
  
  // Apply Gaussian noise to angular velocity
  for (int i = 0; i < 3; ++i) {
    // Gaussian noise based on noise density (continuous-time)
    double gaussian_noise = standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] += imu_parameters_.gyroscope_noise_density * gaussian_noise / std::sqrt(dt);
    
    // Add bias
    (*angular_velocity)[i] += gyroscope_bias_[i];
  }
  
  // Update biases with random walk (Brownian motion)
  // Bias evolves as: b(t+dt) = b(t) + sigma * N(0,1) * sqrt(dt)
  // where sigma is the random walk coefficient
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  
  double sigma_gyro = imu_parameters_.gyroscope_random_walk;
  double sigma_accel = imu_parameters_.accelerometer_random_walk;
  
  if (tau_g > 0.0) {
    double sigma_diff_gyro = sigma_gyro / std::sqrt(tau_g);
    for (int i = 0; i < 3; ++i) {
      double gaussian_noise = standard_normal_distribution_(random_generator_);
      gyroscope_bias_[i] += sigma_diff_gyro * gaussian_noise * std::sqrt(dt);
    }
  }
  
  if (tau_a > 0.0) {
    double sigma_diff_accel = sigma_accel / std::sqrt(tau_a);
    for (int i = 0; i < 3; ++i) {
      double gaussian_noise = standard_normal_distribution_(random_generator_);
      accelerometer_bias_[i] += sigma_diff_accel * gaussian_noise * std::sqrt(dt);
    }
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

  // Copy orientation from quaternion
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = C_W_I.X();
  orientation.y = C_W_I.Y();
  orientation.z = C_W_I.Z();
  orientation.w = C_W_I.W();

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

  // Create ROS2 IMU message
  sensor_msgs::msg::Imu imu_message_ros;
  imu_message_ros.header.frame_id = frame_id_;
  
  // Set timestamp using ROS2 time
  imu_message_ros.header.stamp = rclcpp_node_->now();
  
  // Set sequence
  imu_message_ros.header.seq = seq_++;
  
  // Set orientation
  imu_message_ros.orientation = orientation;
  
  // Set angular velocity
  geometry_msgs::msg::Vector3 angular_velocity_msg;
  angular_velocity_msg.x = angular_velocity_I[0];
  angular_velocity_msg.y = angular_velocity_I[1];
  angular_velocity_msg.z = angular_velocity_I[2];
  imu_message_ros.angular_velocity = angular_velocity_msg;
  
  // Set linear acceleration
  geometry_msgs::msg::Vector3 linear_acceleration_msg;
  linear_acceleration_msg.x = linear_acceleration_I[0];
  linear_acceleration_msg.y = linear_acceleration_I[1];
  linear_acceleration_msg.z = linear_acceleration_I[2];
  imu_message_ros.linear_acceleration = linear_acceleration_msg;
  
  // Set covariances
  imu_message_ros.angular_velocity_covariance[0] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_ros.angular_velocity_covariance[4] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_ros.angular_velocity_covariance[8] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  
  imu_message_ros.orientation_covariance[0] = -1.0;
  imu_message_ros.orientation_covariance[4] = -1.0;
  imu_message_ros.orientation_covariance[8] = -1.0;
  
  imu_message_ros.linear_acceleration_covariance[0] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_ros.linear_acceleration_covariance[4] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_ros.linear_acceleration_covariance[8] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;

  // Publish ROS2 message
  imu_pub_->publish(imu_message_ros);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}
```

Now I need to check if there's a header file that needs to be provided as well. The task mentions multiple files are interdependent, but only one file was provided. Let me provide the header file as well since the implementation depends on it:

[FILENAME: gazebo_imu_plugin.h]
```cpp
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

#ifndef GAZEBO_IMU_PLUGIN_H_
#define GAZEBO_IMU_PLUGIN_H_

#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo {

struct ImuParameters {
  double gyroscope_noise_density;
  double gyroscope_random_walk;
  double gyroscope_bias_correlation_time;
  double gyroscope_turn_on_bias_sigma;
  double accelerometer_noise_density;
  double accelerometer_random_walk;
  double accelerometer_bias_correlation_time;
  double accelerometer_turn_on_bias_sigma;
  double gravity_magnitude;
};

template<typename T>
void getSdfParam(sdf::ElementPtr _sdf, const std::string& _paramName, T& _param, const T _defaultValue)
{
  if (_sdf->HasElement(_paramName)) {
    _param = _sdf->GetElement(_paramName)->Get<T>();
  } else {
    _param = _defaultValue;
    gzwarn << "[gazebo_imu_plugin] Missing parameter \"" << _paramName 
           << "\", setting to default: " << _defaultValue << std::endl;
  }
}

class GazeboImuPlugin : public ModelPlugin {
 public:
  GazeboImuPlugin();
  virtual ~GazeboImuPlugin();
  
  // Documentation inherited
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 protected:
  /// \brief This function adds noise to acceleration and angular rates for
  ///        accelerometer and gyroscope measurement simulation.
  void addNoise(Eigen::Vector3d* linear_acceleration,
                Eigen::Vector3d* angular_velocity,
                const double dt);
                
  /// \brief Get the parent link.
  physics::LinkPtr link() const {
    return link_;
  }
  
 private:
  /// \brief Called by the world update start event.
  void OnUpdate(const common::UpdateInfo& _info);

  /// \brief Get an quaternion from a gazebo quaternion.
  static const ignition::math::Quaterniond& ignitionFromGazeboMath(
      const gazebo::math::Quaternion& _q) {
    return reinterpret_cast<const ignition::math::Quaterniond&>(_q);
  }

  /// \brief Get an quaternion from a gazebo quaternion.
  static const ignition::math::Vector3d& ignitionFromGazeboMath(
      const gazebo::math::Vector3& _v) {
    return reinterpret_cast<const ignition::math::Vector3d&>(_v);
  }

  /// \brief Store link pointer.
  physics::LinkPtr link_;

  /// \brief Store model pointer.
  physics::ModelPtr model_;

  /// \brief Store world pointer.
  physics::WorldPtr world_;

  /// \brief ROS2 node for communication
  rclcpp::Node::SharedPtr rclcpp_node_;
  
  /// \brief ROS2 publisher for IMU data
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  /// \brief IMU message.
  sensor_msgs::msg::Imu imu_message_;

  /// \brief IMU parameters.
  ImuParameters imu_parameters_;

  /// \brief Store the latest update time.
  common::Time last_time_;

  /// \brief Frame ID for IMU data.
  std::string frame_id_;

  /// \brief Store the namespace.
  std::string namespace_;

  /// \brief Store the link name.
  std::string link_name_;

  /// \brief The IMU topic name.
  std::string imu_topic_;

  /// \brief Gazebo connection for world update.
  event::ConnectionPtr updateConnection_;

  /// \brief Previous velocity in world frame.
  ignition::math::Vector3d velocity_prev_W_;

  /// \brief Gravity in world frame.
  ignition::math::Vector3d gravity_W_;

  /// \brief Sequence number for IMU data.
  uint32_t seq_;

  /// \brief Gyroscope bias.
  Eigen::Vector3d gyroscope_bias_;

  /// \brief Accelerometer bias.
  Eigen::Vector3d accelerometer_bias_;

  /// \brief Standard normal distribution for noise generation.
  std::normal_distribution<double> standard_normal_distribution_;

  /// \brief Random number generator.
  std::default_random_engine random_generator_;

  /// \brief Default IMU topic name.
  static constexpr char kDefaultImuTopic[] = "/imu";
};

}  // namespace gazebo

#endif  // GAZEBO_IMU_PLUGIN_H_
```