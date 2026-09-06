/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */
/* Migrated to ROS 2 Humble */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode() = default;

private:
  rcl_interfaces::msg::SetParametersResult
  on_params_set(const std::vector<rclcpp::Parameter> & params);

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Internal state members
  int min_particles_;
  int max_particles_;
  int max_beams_;
  int resample_interval_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;
  std::string laser_model_type_str_;
  std::string odom_model_type_str_;

  double update_min_d_;
  double update_min_a_;
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
  double alpha_slow_, alpha_fast_;
  double z_hit_, z_short_, z_max_, z_rand_;
  double sigma_hit_, lambda_short_;
  double laser_likelihood_max_dist_;
  double laser_min_range_;
  double laser_max_range_;
  double pf_err_, pf_z_;
  double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;

  bool do_beamskip_;
  bool tf_broadcast_;
  bool selective_resampling_;
  bool force_update_after_initialpose_;
  bool force_update_after_set_map_;
  bool use_map_topic_;
  bool first_map_only_;

  double transform_tolerance_;
  double gui_publish_period_;
  double save_pose_period_;

  double init_pose_x_;
  double init_pose_y_;
  double init_pose_a_;
  double init_cov_xx_;
  double init_cov_yy_;
  double init_cov_aa_;

  double std_warn_level_x_;
  double std_warn_level_y_;
  double std_warn_level_yaw_;
};

AmclNode::AmclNode()
: Node("amcl")
{
  // Declare all parameters with explicit template types and defaults
  this->declare_parameter<int>("min_particles", 100);
  this->declare_parameter<int>("max_particles", 5000);
  this->declare_parameter<int>("max_beams", 30);
  this->declare_parameter<int>("resample_interval", 2);

  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("laser_model_type", "likelihood_field");
  this->declare_parameter<std::string>("odom_model_type", "diff");

  this->declare_parameter<double>("update_min_d", 0.2);
  this->declare_parameter<double>("update_min_a", 0.5236);
  this->declare_parameter<double>("alpha1", 0.2);
  this->declare_parameter<double>("alpha2", 0.2);
  this->declare_parameter<double>("alpha3", 0.2);
  this->declare_parameter<double>("alpha4", 0.2);
  this->declare_parameter<double>("alpha5", 0.2);
  this->declare_parameter<double>("alpha_slow", 0.001);
  this->declare_parameter<double>("alpha_fast", 0.1);
  this->declare_parameter<double>("z_hit", 0.95);
  this->declare_parameter<double>("z_short", 0.1);
  this->declare_parameter<double>("z_max", 0.05);
  this->declare_parameter<double>("z_rand", 0.05);
  this->declare_parameter<double>("sigma_hit", 0.2);
  this->declare_parameter<double>("lambda_short", 0.1);
  this->declare_parameter<double>("laser_likelihood_max_dist", 2.0);
  this->declare_parameter<double>("laser_min_range", -1.0);
  this->declare_parameter<double>("laser_max_range", -1.0);
  this->declare_parameter<double>("pf_err", 0.01);
  this->declare_parameter<double>("pf_z", 0.99);
  this->declare_parameter<double>("beam_skip_distance", 0.5);
  this->declare_parameter<double>("beam_skip_threshold", 0.3);
  this->declare_parameter<double>("beam_skip_error_threshold", 0.9);
  this->declare_parameter<double>("transform_tolerance", 0.1);
  this->declare_parameter<double>("gui_publish_period", -1.0);
  this->declare_parameter<double>("save_pose_period", 0.5);

  this->declare_parameter<bool>("do_beamskip", false);
  this->declare_parameter<bool>("tf_broadcast", true);
  this->declare_parameter<bool>("selective_resampling", false);
  this->declare_parameter<bool>("force_update_after_initialpose", false);
  this->declare_parameter<bool>("force_update_after_set_map", false);
  this->declare_parameter<bool>("use_map_topic", false);
  this->declare_parameter<bool>("first_map_only", false);

  this->declare_parameter<double>("initial_pose_x", 0.0);
  this->declare_parameter<double>("initial_pose_y", 0.0);
  this->declare_parameter<double>("initial_pose_a", 0.0);
  this->declare_parameter<double>("initial_cov_xx", 0.25);
  this->declare_parameter<double>("initial_cov_yy", 0.25);
  this->declare_parameter<double>("initial_cov_aa", 0.068539);

  this->declare_parameter<double>("std_warn_level_x", 0.2);
  this->declare_parameter<double>("std_warn_level_y", 0.2);
  this->declare_parameter<double>("std_warn_level_yaw", 0.2);

  // Read initial values into member variables
  min_particles_ = this->get_parameter("min_particles").as_int();
  max_particles_ = this->get_parameter("max_particles").as_int();
  max_beams_ = this->get_parameter("max_beams").as_int();
  resample_interval_ = this->get_parameter("resample_interval").as_int();

  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  laser_model_type_str_ = this->get_parameter("laser_model_type").as_string();
  odom_model_type_str_ = this->get_parameter("odom_model_type").as_string();

  update_min_d_ = this->get_parameter("update_min_d").as_double();
  update_min_a_ = this->get_parameter("update_min_a").as_double();
  alpha1_ = this->get_parameter("alpha1").as_double();
  alpha2_ = this->get_parameter("alpha2").as_double();
  alpha3_ = this->get_parameter("alpha3").as_double();
  alpha4_ = this->get_parameter("alpha4").as_double();
  alpha5_ = this->get_parameter("alpha5").as_double();
  alpha_slow_ = this->get_parameter("alpha_slow").as_double();
  alpha_fast_ = this->get_parameter("alpha_fast").as_double();
  z_hit_ = this->get_parameter("z_hit").as_double();
  z_short_ = this->get_parameter("z_short").as_double();
  z_max_ = this->get_parameter("z_max").as_double();
  z_rand_ = this->get_parameter("z_rand").as_double();
  sigma_hit_ = this->get_parameter("sigma_hit").as_double();
  lambda_short_ = this->get_parameter("lambda_short").as_double();
  laser_likelihood_max_dist_ = this->get_parameter("laser_likelihood_max_dist").as_double();
  laser_min_range_ = this->get_parameter("laser_min_range").as_double();
  laser_max_range_ = this->get_parameter("laser_max_range").as_double();
  pf_err_ = this->get_parameter("pf_err").as_double();
  pf_z_ = this->get_parameter("pf_z").as_double();
  beam_skip_distance_ = this->get_parameter("beam_skip_distance").as_double();
  beam_skip_threshold_ = this->get_parameter("beam_skip_threshold").as_double();
  beam_skip_error_threshold_ = this->get_parameter("beam_skip_error_threshold").as_double();
  transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();
  gui_publish_period_ = this->get_parameter("gui_publish_period").as_double();
  save_pose_period_ = this->get_parameter("save_pose_period").as_double();

  do_beamskip_ = this->get_parameter("do_beamskip").get_parameter_value().get<bool>();
  tf_broadcast_ = this->get_parameter("tf_broadcast").get_parameter_value().get<bool>();
  selective_resampling_ = this->get_parameter("selective_resampling").get_parameter_value().get<bool>();
  force_update_after_initialpose_ = this->get_parameter("force_update_after_initialpose").get_parameter_value().get<bool>();
  force_update_after_set_map_ = this->get_parameter("force_update_after_set_map").get_parameter_value().get<bool>();
  use_map_topic_ = this->get_parameter("use_map_topic").get_parameter_value().get<bool>();
  first_map_only_ = this->get_parameter("first_map_only").get_parameter_value().get<bool>();

  init_pose_x_ = this->get_parameter("initial_pose_x").as_double();
  init_pose_y_ = this->get_parameter("initial_pose_y").as_double();
  init_pose_a_ = this->get_parameter("initial_pose_a").as_double();
  init_cov_xx_ = this->get_parameter("initial_cov_xx").as_double();
  init_cov_yy_ = this->get_parameter("initial_cov_yy").as_double();
  init_cov_aa_ = this->get_parameter("initial_cov_aa").as_double();

  std_warn_level_x_ = this->get_parameter("std_warn_level_x").as_double();
  std_warn_level_y_ = this->get_parameter("std_warn_level_y").as_double();
  std_warn_level_yaw_ = this->get_parameter("std_warn_level_yaw").as_double();

  // Register the dynamic parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AmclNode::on_params_set, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AMCL node initialized with ROS 2 unified parameter API.");
}

rcl_interfaces::msg::SetParametersResult
AmclNode::on_params_set(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "OK";

  // Stage proposed values for atomic validation
  int proposed_min_particles = min_particles_;
  int proposed_max_particles = max_particles_;
  int proposed_max_beams = max_beams_;
  int proposed_resample_interval = resample_interval_;

  std::string proposed_odom_frame_id = odom_frame_id_;
  std::string proposed_base_frame_id = base_frame_id_;
  std::string proposed_global_frame_id = global_frame_id_;
  std::string proposed_laser_model_type = laser_model_type_str_;
  std::string proposed_odom_model_type = odom_model_type_str_;

  double proposed_update_min_d = update_min_d_;
  double proposed_update_min_a = update_min_a_;
  double proposed_alpha1 = alpha1_;
  double proposed_alpha2 = alpha2_;
  double proposed_alpha3 = alpha3_;
  double proposed_alpha4 = alpha4_;
  double proposed_alpha5 = alpha5_;
  double proposed_alpha_slow = alpha_slow_;
  double proposed_alpha_fast = alpha_fast_;
  double proposed_z_hit = z_hit_;
  double proposed_z_short = z_short_;
  double proposed_z_max = z_max_;
  double proposed_z_rand = z_rand_;
  double proposed_sigma_hit = sigma_hit_;
  double proposed_lambda_short = lambda_short_;
  double proposed_laser_likelihood_max_dist = laser_likelihood_max_dist_;
  double proposed_laser_min_range = laser_min_range_;
  double proposed_laser_max_range = laser_max_range_;
  double proposed_pf_err = pf_err_;
  double proposed_pf_z = pf_z_;
  double proposed_transform_tolerance = transform_tolerance_;

  // First pass: collect proposed values
  for (const auto & param : params) {
    const std::string & name = param.get_name();

    if (name == "min_particles") {
      proposed_min_particles = param.as_int();
    } else if (name == "max_particles") {
      proposed_max_particles = param.as_int();
    } else if (name == "max_beams") {
      proposed_max_beams = param.as_int();
    } else if (name == "resample_interval") {
      proposed_resample_interval = param.as_int();
    } else if (name == "odom_frame_id") {
      proposed_odom_frame_id = param.as_string();
    } else if (name == "base_frame_id") {
      proposed_base_frame_id = param.as_string();
    } else if (name == "global_frame_id") {
      proposed_global_frame_id = param.as_string();
    } else if (name == "laser_model_type") {
      proposed_laser_model_type = param.as_string();
    } else if (name == "odom_model_type") {
      proposed_odom_model_type = param.as_string();
    } else if (name == "update_min_d") {
      proposed_update_min_d = param.as_double();
    } else if (name == "update_min_a") {
      proposed_update_min_a = param.as_double();
    } else if (name == "alpha1") {
      proposed_alpha1 = param.as_double();
    } else if (name == "alpha2") {
      proposed_alpha2 = param.as_double();
    } else if (name == "alpha3") {
      proposed_alpha3 = param.as_double();
    } else if (name == "alpha4") {
      proposed_alpha4 = param.as_double();
    } else if (name == "alpha5") {
      proposed_alpha5 = param.as_double();
    } else if (name == "alpha_slow") {
      proposed_alpha_slow = param.as_double();
    } else if (name == "alpha_fast") {
      proposed_alpha_fast = param.as_double();
    } else if (name == "z_hit") {
      proposed_z_hit = param.as_double();
    } else if (name == "z_short") {
      proposed_z_short = param.as_double();
    } else if (name == "z_max") {
      proposed_z_max = param.as_double();
    } else if (name == "z_rand") {
      proposed_z_rand = param.as_double();
    } else if (name == "sigma_hit") {
      proposed_sigma_hit = param.as_double();
    } else if (name == "lambda_short") {
      proposed_lambda_short = param.as_double();
    } else if (name == "laser_likelihood_max_dist") {
      proposed_laser_likelihood_max_dist = param.as_double();
    } else if (name == "laser_min_range") {
      proposed_laser_min_range = param.as_double();
    } else if (name == "laser_max_range") {
      proposed_laser_max_range = param.as_double();
    } else if (name == "pf_err") {
      proposed_pf_err = param.as_double();
    } else if (name == "pf_z") {
      proposed_pf_z = param.as_double();
    } else if (name == "transform_tolerance") {
      proposed_transform_tolerance = param.as_double();
    }
  }

  // Validation: min_particles must not exceed max_particles
  if (proposed_min_particles > proposed_max_particles) {
    result.successful = false;
    result.reason = "min_particles must not be greater than max_particles";
    return result;
  }

  // All validations passed — apply updates atomically
  min_particles_ = proposed_min_particles;
  max_particles_ = proposed_max_particles;
  max_beams_ = proposed_max_beams;
  resample_interval_ = proposed_resample_interval;

  odom_frame_id_ = proposed_odom_frame_id;
  base_frame_id_ = proposed_base_frame_id;
  global_frame_id_ = proposed_global_frame_id;
  laser_model_type_str_ = proposed_laser_model_type;
  odom_model_type_str_ = proposed_odom_model_type;

  update_min_d_ = proposed_update_min_d;
  update_min_a_ = proposed_update_min_a;
  alpha1_ = proposed_alpha1;
  alpha2_ = proposed_alpha2;
  alpha3_ = proposed_alpha3;
  alpha4_ = proposed_alpha4;
  alpha5_ = proposed_alpha5;
  alpha_slow_ = proposed_alpha_slow;
  alpha_fast_ = proposed_alpha_fast;
  z_hit_ = proposed_z_hit;
  z_short_ = proposed_z_short;
  z_max_ = proposed_z_max;
  z_rand_ = proposed_z_rand;
  sigma_hit_ = proposed_sigma_hit;
  lambda_short_ = proposed_lambda_short;
  laser_likelihood_max_dist_ = proposed_laser_likelihood_max_dist;
  laser_min_range_ = proposed_laser_min_range;
  laser_max_range_ = proposed_laser_max_range;
  pf_err_ = proposed_pf_err;
  pf_z_ = proposed_pf_z;
  transform_tolerance_ = proposed_transform_tolerance;

  RCLCPP_INFO(this->get_logger(), "Parameters updated successfully.");
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}