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

#ifndef GAZEBO_IMU_PLUGIN_H
#define GAZEBO_IMU_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo {

// Default values for use with IIM42653 IMU
static constexpr double kDefaultGyroscopeNoiseDensity =
    0.0008726646;
static constexpr double kDefaultGyroscopeRandomWalk =
    0.0;
static constexpr double kDefaultGyroscopeBiasCorrelationTime =
    1000.0;
static constexpr double kDefaultGyroscopeTurnOnBiasSigma =
    0.0;
static constexpr double kDefaultAccelerometerNoiseDensity =
    0.00637;
static constexpr double kDefaultAccelerometerRandomWalk =
    0.0;
static constexpr double kDefaultAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAccelerometerTurnOnBiasSigma =
    0.0;
static constexpr double kDefaultGravityMagnitude = 9.8068;

static const std::string kDefaultImuTopic = "imu";

template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[gazebo_imu_plugin] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

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

  ImuParameters()
      : gyroscope_noise_density(kDefaultGyroscopeNoiseDensity),
        gyroscope_random_walk(kDefaultGyroscopeRandomWalk),
        gyroscope_bias_correlation_time(
            kDefaultGyroscopeBiasCorrelationTime),
        gyroscope_turn_on_bias_sigma(kDefaultGyroscopeTurnOnBiasSigma),
        accelerometer_noise_density(kDefaultAccelerometerNoiseDensity),
        accelerometer_random_walk(kDefaultAccelerometerRandomWalk),
        accelerometer_bias_correlation_time(
            kDefaultAccelerometerBiasCorrelationTime),
        accelerometer_turn_on_bias_sigma(
            kDefaultAccelerometerTurnOnBiasSigma),
        gravity_magnitude(kDefaultGravityMagnitude) {}
};

class GazeboImuPlugin : public ModelPlugin {
 public:
  GazeboImuPlugin();
  ~GazeboImuPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void addNoise(
      Eigen::Vector3d* linear_acceleration,
      Eigen::Vector3d* angular_velocity,
      const double dt);

  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string imu_topic_;
  std::string frame_id_;
  std::string link_name_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  // ROS 2 node and publisher
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::Imu imu_message_;

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  Eigen::Vector3d gyroscope_bias_;
  Eigen::Vector3d accelerometer_bias_;

  ImuParameters imu_parameters_;

  uint64_t seq_ = 0;
};
}  // namespace gazebo

#endif  // GAZEBO_IMU_PLUGIN_H