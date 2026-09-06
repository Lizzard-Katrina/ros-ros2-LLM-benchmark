/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <imm_ukf_pda/imm_ukf_pda.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ImmUkfPda::ImmUkfPda()
  : Node("imm_ukf_pda_tracker"),
    target_id_(0),
    init_(false),
    frame_count_(0),
    has_subscribed_vectormap_(false)
{
  this->declare_parameter<std::string>("tracking_frame", "map");
  this->declare_parameter<int>("life_time_threshold", 8);
  this->declare_parameter<double>("gating_threshold", 9.22);
  this->declare_parameter<double>("gate_probability", 0.99);
  this->declare_parameter<double>("detection_probability", 0.9);
  this->declare_parameter<double>("static_velocity_threshold", 0.5);
  this->declare_parameter<int>("static_num_history_threshold", 3);
  this->declare_parameter<double>("prevent_explosion_threshold", 1000.0);
  this->declare_parameter<double>("merge_distance_threshold", 0.5);
  this->declare_parameter<bool>("use_sukf", false);

  this->declare_parameter<bool>("use_vectormap", false);
  this->declare_parameter<double>("lane_direction_chi_threshold", 2.71);
  this->declare_parameter<double>("nearest_lane_distance_threshold", 1.0);
  this->declare_parameter<std::string>("vectormap_frame", "map");

  this->declare_parameter<bool>("is_benchmark", false);
  this->declare_parameter<std::string>("kitti_data_dir", "");

  this->get_parameter("tracking_frame", tracking_frame_);
  this->get_parameter("life_time_threshold", life_time_threshold_);
  this->get_parameter("gating_threshold", gating_threshold_);
  this->get_parameter("gate_probability", gate_probability_);
  this->get_parameter("detection_probability", detection_probability_);
  this->get_parameter("static_velocity_threshold", static_velocity_threshold_);
  this->get_parameter("static_num_history_threshold", static_num_history_threshold_);
  this->get_parameter("prevent_explosion_threshold", prevent_explosion_threshold_);
  this->get_parameter("merge_distance_threshold", merge_distance_threshold_);
  this->get_parameter("use_sukf", use_sukf_);

  this->get_parameter("use_vectormap", use_vectormap_);
  this->get_parameter("lane_direction_chi_threshold", lane_direction_chi_threshold_);
  this->get_parameter("nearest_lane_distance_threshold", nearest_lane_distance_threshold_);
  this->get_parameter("vectormap_frame", vectormap_frame_);

  this->get_parameter("is_benchmark", is_benchmark_);
  this->get_parameter("kitti_data_dir", kitti_data_dir_);

  if (is_benchmark_)
  {
    result_file_path_ = kitti_data_dir_ + "benchmark_results.txt";
    std::remove(result_file_path_.c_str());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ImmUkfPda::run()
{
  pub_object_array_ = this->create_publisher<autoware_msgs::msg::DetectedObjectArray>("objects_out", 1);
  sub_detected_array_ = this->create_subscription<autoware_msgs::msg::DetectedObjectArray>(
    "objects_in", 1, std::bind(&ImmUkfPda::callback, this, std::placeholders::_1));
}

void ImmUkfPda::callback(const autoware_msgs::msg::DetectedObjectArray::SharedPtr input)
{
  input_header_ = input->header;

  if (use_vectormap_)
  {
    checkVectormapSubscription();
  }

  bool success = updateNecessaryTransform();
  if (!success)
  {
    RCLCPP_INFO(this->get_logger(), "Could not find coordiante transformation");
    return;
  }

  autoware_msgs::msg::DetectedObjectArray transformed_input;
  autoware_msgs::msg::DetectedObjectArray detected_objects_output;
  transformPoseToGlobal(*input, transformed_input);
  tracker(transformed_input, detected_objects_output);
  transformPoseToLocal(detected_objects_output);

  pub_object_array_->publish(detected_objects_output);

  if (is_benchmark_)
  {
    dumpResultText(detected_objects_output);
  }
}

void ImmUkfPda::checkVectormapSubscription()
{
  if (use_vectormap_ && !has_subscribed_vectormap_)
  {
    if (lanes_.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Has not subscribed vectormap");
    }
    else
    {
      has_subscribed_vectormap_ = true;
    }
  }
}

bool ImmUkfPda::updateNecessaryTransform()
{
  bool success = true;
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(
      tracking_frame_, input_header_.frame_id,
      tf2::timeFromSec(rclcpp::Time(input_header_.stamp).seconds()),
      tf2::durationFromSec(1.0));

    tf2::Transform tf2_local2global;
    tf2::fromMsg(transform_stamped.transform, tf2_local2global);
    local2global_.setData(tf2_local2global);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    success = false;
  }

  if (use_vectormap_ && has_subscribed_vectormap_)
  {
    try
    {
      geometry_msgs::msg::TransformStamped vmap_tf;
      vmap_tf = tf_buffer_->lookupTransform(
        vectormap_frame_, tracking_frame_,
        tf2::timeFromSec(rclcpp::Time(input_header_.stamp).seconds()),
        tf2::durationFromSec(1.0));

      tf2::Transform tf2_tracking2lane;
      tf2::fromMsg(vmap_tf.transform, tf2_tracking2lane);
      tracking_frame2lane_frame_.setData(tf2_tracking2lane);

      geometry_msgs::msg::TransformStamped lane2tracking_tf;
      lane2tracking_tf = tf_buffer_->lookupTransform(
        tracking_frame_, vectormap_frame_,
        tf2::timeFromSec(rclcpp::Time(input_header_.stamp).seconds()),
        tf2::durationFromSec(1.0));

      tf2::Transform tf2_lane2tracking;
      tf2::fromMsg(lane2tracking_tf.transform, tf2_lane2tracking);
      lane_frame2tracking_frame_.setData(tf2_lane2tracking);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  return success;
}

void ImmUkfPda::transformPoseToGlobal(const autoware_msgs::msg::DetectedObjectArray& input,
                                      autoware_msgs::msg::DetectedObjectArray& transformed_input)
{
  transformed_input.header = input_header_;
  for (auto const &object: input.objects)
  {
    geometry_msgs::msg::Pose out_pose = getTransformedPose(object.pose, local2global_);

    autoware_msgs::msg::DetectedObject dd;
    dd.header = input.header;
    dd = object;
    dd.pose = out_pose;

    transformed_input.objects.push_back(dd);
  }
}

void ImmUkfPda::transformPoseToLocal(autoware_msgs::msg::DetectedObjectArray& detected_objects_output)
{
  detected_objects_output.header = input_header_;

  tf2::Transform inv_local2global = local2global_.inverse();
  tf2::Stamped<tf2::Transform> global2local;
  global2local.setData(inv_local2global);
  for (auto& object : detected_objects_output.objects)
  {
    geometry_msgs::msg::Pose out_pose = getTransformedPose(object.pose, global2local);
    object.header = input_header_;
    object.pose = out_pose;
  }
}

geometry_msgs::msg::Pose ImmUkfPda::getTransformedPose(const geometry_msgs::msg::Pose& in_pose,
                                                       const tf2::Stamped<tf2::Transform>& tf_stamp)
{
  tf2::Transform transform;
  geometry_msgs::msg::PoseStamped out_pose;
  transform.setOrigin(tf2::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
      tf2::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  tf2::Transform result = tf_stamp * transform;
  tf2::Vector3 origin = result.getOrigin();
  tf2::Quaternion rotation = result.getRotation();
  out_pose.pose.position.x = origin.x();
  out_pose.pose.position.y = origin.y();
  out_pose.pose.position.z = origin.z();
  out_pose.pose.orientation.x = rotation.x();
  out_pose.pose.orientation.y = rotation.y();
  out_pose.pose.orientation.z = rotation.z();
  out_pose.pose.orientation.w = rotation.w();
  return out_pose.pose;
}

void ImmUkfPda::measurementValidation(const autoware_msgs::msg::DetectedObjectArray& input, UKF& target,
                                      const bool second_init, const Eigen::VectorXd& max_det_z,
                                      const Eigen::MatrixXd& max_det_s,
                                      std::vector<autoware_msgs::msg::DetectedObject>& object_vec,
                                      std::vector<bool>& matching_vec)
{
  bool exists_smallest_nis_object = false;
  double smallest_nis = std::numeric_limits<double>::max();
  int smallest_nis_ind = 0;
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double x = input.objects[i].pose.position.x;
    double y = input.objects[i].pose.position.y;

    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas << x, y;

    Eigen::VectorXd diff = meas - max_det_z;
    double nis = diff.transpose() * max_det_s.inverse() * diff;

    if (nis < gating_threshold_)
    {
      if (nis < smallest_nis)
      {
        smallest_nis = nis;
        target.object_ = input.objects[i];
        smallest_nis_ind = i;
        exists_smallest_nis_object = true;
      }
    }
  }
  if (exists_smallest_nis_object)
  {
    matching_vec[smallest_nis_ind] = true;
    if (use_vectormap_ && has_subscribed_vectormap_)
    {
      autoware_msgs::msg::DetectedObject direction_updated_object;
      bool use_direction_meas =
          updateDirection(smallest_nis, target.object_, direction_updated_object, target);
      if (use_direction_meas)
      {
        object_vec.push_back(direction_updated_object);
      }
      else
      {
        object_vec.push_back(target.object_);
      }
    }
    else
    {
      object_vec.push_back(target.object_);
    }
  }
}

bool ImmUkfPda::updateDirection(const double smallest_nis, const autoware_msgs::msg::DetectedObject& in_object,
                                    autoware_msgs::msg::DetectedObject& out_object, UKF& target)
{
  bool use_lane_direction = false;
  target.is_direction_cv_available_ = false;
  target.is_direction_ctrv_available_ = false;
  bool get_lane_success = storeObjectWithNearestLaneDirection(in_object, out_object);
  if (!get_lane_success)
  {
    return use_lane_direction;
  }
  target.checkLaneDirectionAvailability(out_object, lane_direction_chi_threshold_, use_sukf_);
  if (target.is_direction_cv_available_ || target.is_direction_ctrv_available_)
  {
    use_lane_direction = true;
  }
  return use_lane_direction;
}

bool ImmUkfPda::storeObjectWithNearestLaneDirection(const autoware_msgs::msg::DetectedObject& in_object,
                                                 autoware_msgs::msg::DetectedObject& out_object)
{
  geometry_msgs::msg::Pose lane_frame_pose = getTransformedPose(in_object.pose, tracking_frame2lane_frame_);
  double min_dist = std::numeric_limits<double>::max();

  double min_yaw = 0;
  for (auto const& lane : lanes_)
  {
    (void)lane;
    // vectormap lookup omitted in ROS 2 migration (no vector_map dependency)
  }

  bool success = false;
  if (min_dist < nearest_lane_distance_threshold_)
  {
    success = true;
  }
  else
  {
    return success;
  }

  tf2::Quaternion map_quat;
  map_quat.setRPY(0, 0, min_yaw);
  tf2::Matrix3x3 map_matrix(map_quat);

  tf2::Quaternion rotation_quat = lane_frame2tracking_frame_.getRotation();
  tf2::Matrix3x3 rotation_matrix(rotation_quat);

  tf2::Matrix3x3 rotated_matrix = rotation_matrix * map_matrix;
  double roll, pitch, yaw;
  rotated_matrix.getRPY(roll, pitch, yaw);

  out_object = in_object;
  out_object.angle = yaw;
  return success;
}

void ImmUkfPda::updateTargetWithAssociatedObject(const std::vector<autoware_msgs::msg::DetectedObject>& object_vec,
                                                 UKF& target)
{
  target.lifetime_++;
  if (!target.object_.label.empty() && target.object_.label != "unknown")
  {
    target.label_ = target.object_.label;
  }
  updateTrackingNum(object_vec, target);
  if (target.tracking_num_ == TrackingState::Stable || target.tracking_num_ == TrackingState::Occlusion)
  {
    target.is_stable_ = true;
  }
}

void ImmUkfPda::updateBehaviorState(const UKF& target, const bool use_sukf, autoware_msgs::msg::DetectedObject& object)
{
  if (use_sukf)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CV;
  }
  else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else
  {
    object.behavior_state = MotionModel::RM;
  }
}

void ImmUkfPda::initTracker(const autoware_msgs::msg::DetectedObjectArray& input, double timestamp)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double px = input.objects[i].pose.position.x;
    double py = input.objects[i].pose.position.y;
    Eigen::VectorXd init_meas = Eigen::VectorXd(2);
    init_meas << px, py;

    UKF ukf;
    ukf.initialize(init_meas, timestamp, target_id_);
    targets_.push_back(ukf);
    target_id_++;
  }
  timestamp_ = timestamp;
  init_ = true;
}

void ImmUkfPda::secondInit(UKF& target, const std::vector<autoware_msgs::msg::DetectedObject>& object_vec, double dt)
{
  if (object_vec.size() == 0)
  {
    target.tracking_num_ = TrackingState::Die;
    return;
  }
  target.init_meas_ << target.x_merge_(0), target.x_merge_(1);

  double target_x = object_vec[0].pose.position.x;
  double target_y = object_vec[0].pose.position.y;
  double target_diff_x = target_x - target.x_merge_(0);
  double target_diff_y = target_y - target.x_merge_(1);
  double target_yaw = atan2(target_diff_y, target_diff_x);
  double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
  double target_v = dist / dt;

  while (target_yaw > M_PI)
    target_yaw -= 2. * M_PI;
  while (target_yaw < -M_PI)
    target_yaw += 2. * M_PI;

  target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
  target.x_merge_(1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
  target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
  target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;

  target.tracking_num_++;
  return;
}

void ImmUkfPda::updateTrackingNum(const std::vector<autoware_msgs::msg::DetectedObject>& object_vec, UKF& target)
{
  if (object_vec.size() > 0)
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }
  else
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Die;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }

  return;
}

bool ImmUkfPda::probabilisticDataAssociation(const autoware_msgs::msg::DetectedObjectArray& input, const double dt,
                                             std::vector<bool>& matching_vec,
                                             std::vector<autoware_msgs::msg::DetectedObject>& object_vec, UKF& target)
{
  double det_s = 0;
  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;
  bool success = true;

  if (use_sukf_)
  {
    max_det_z = target.z_pred_ctrv_;
    max_det_s = target.s_ctrv_;
    det_s = max_det_s.determinant();
  }
  else
  {
    target.findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();
  }

  if (std::isnan(det_s) || det_s > prevent_explosion_threshold_)
  {
    target.tracking_num_ = TrackingState::Die;
    success = false;
    return success;
  }

  bool is_second_init;
  if (target.tracking_num_ == TrackingState::Init)
  {
    is_second_init = true;
  }
  else
  {
    is_second_init = false;
  }

  measurementValidation(input, target, is_second_init, max_det_z, max_det_s, object_vec, matching_vec);

  if (is_second_init)
  {
    secondInit(target, object_vec, dt);
    success = false;
    return success;
  }

  updateTargetWithAssociatedObject(object_vec, target);

  if (target.tracking_num_ == TrackingState::Die)
  {
    success = false;
    return success;
  }
  return success;
}

void ImmUkfPda::makeNewTargets(const double timestamp, const autoware_msgs::msg::DetectedObjectArray& input,
                               const std::vector<bool>& matching_vec)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    if (matching_vec[i] == false)
    {
      double px = input.objects[i].pose.position.x;
      double py = input.objects[i].pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_);
      ukf.object_ = input.objects[i];
      targets_.push_back(ukf);
      target_id_++;
    }
  }
}

void ImmUkfPda::staticClassification()
{
  for (size_t i = 0; i < targets_.size(); i++)
  {
    double current_velocity = std::abs(targets_[i].x_merge_(2));
    targets_[i].vel_history_.push_back(current_velocity);
    if (targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_threshold_)
    {
      int index = 0;
      double sum_vel = 0;
      double avg_vel = 0;
      for (auto rit = targets_[i].vel_history_.rbegin(); index < static_num_history_threshold_; ++rit)
      {
        index++;
        sum_vel += *rit;
      }
      avg_vel = double(sum_vel / static_num_history_threshold_);

      if (avg_vel < static_velocity_threshold_ && current_velocity < static_velocity_threshold_)
      {
        targets_[i].is_static_ = true;
      }
    }
  }
}

bool ImmUkfPda::arePointsClose(const geometry_msgs::msg::Point& in_point_a,
                                const geometry_msgs::msg::Point& in_point_b,
                                float in_radius)
{
  return (fabs(in_point_a.x - in_point_b.x) <= in_radius) && (fabs(in_point_a.y - in_point_b.y) <= in_radius);
}

bool ImmUkfPda::arePointsEqual(const geometry_msgs::msg::Point& in_point_a,
                               const geometry_msgs::msg::Point& in_point_b)
{
  return arePointsClose(in_point_a, in_point_b, CENTROID_DISTANCE);
}

bool ImmUkfPda::isPointInPool(const std::vector<geometry_msgs::msg::Point>& in_pool,
                              const geometry_msgs::msg::Point& in_point)
{
  for (size_t j = 0; j < in_pool.size(); j++)
  {
    if (arePointsEqual(in_pool[j], in_point))
    {
      return true;
    }
  }
  return false;
}

autoware_msgs::msg::DetectedObjectArray
ImmUkfPda::removeRedundantObjects(const autoware_msgs::msg::DetectedObjectArray& in_detected_objects,
                            const std::vector<size_t> in_tracker_indices)
{
  if (in_detected_objects.objects.size() != in_tracker_indices.size())
    return in_detected_objects;

  autoware_msgs::msg::DetectedObjectArray resulting_objects;
  resulting_objects.header = in_detected_objects.header;

  std::vector<geometry_msgs::msg::Point> centroids;
  for (size_t i = 0; i < in_detected_objects.objects.size(); i++)
  {
    if (!isPointInPool(centroids, in_detected_objects.objects[i].pose.position))
    {
      centroids.push_back(in_detected_objects.objects[i].pose.position);
    }
  }

  std::vector<std::vector<size_t>> matching_objects(centroids.size());
  for (size_t k = 0; k < in_detected_objects.objects.size(); k++)
  {
    const auto& object = in_detected_objects.objects[k];
    for (size_t i = 0; i < centroids.size(); i++)
    {
      if (arePointsClose(object.pose.position, centroids[i], merge_distance_threshold_))
      {
        matching_objects[i].push_back(k);
      }
    }
  }

  for (size_t i = 0; i < matching_objects.size(); i++)
  {
    size_t oldest_object_index = 0;
    int oldest_lifespan = -1;
    std::string best_label;
    for (size_t j = 0; j < matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
      if (current_lifespan > oldest_lifespan)
      {
        oldest_lifespan = current_lifespan;
        oldest_object_index = current_index;
      }
      if (!targets_[in_tracker_indices[current_index]].label_.empty() &&
        targets_[in_tracker_indices[current_index]].label_ != "unknown")
      {
        best_label = targets_[in_tracker_indices[current_index]].label_;
      }
    }
    for (size_t j = 0; j < matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      if (current_index != oldest_object_index)
      {
        targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
      }
    }
    autoware_msgs::msg::DetectedObject best_object;
    best_object = in_detected_objects.objects[oldest_object_index];
    if (best_label != "unknown" && !best_label.empty())
    {
      best_object.label = best_label;
    }

    resulting_objects.objects.push_back(best_object);
  }

  return resulting_objects;
}

void ImmUkfPda::makeOutput(const autoware_msgs::msg::DetectedObjectArray& input,
                           const std::vector<bool>& matching_vec,
                           autoware_msgs::msg::DetectedObjectArray& detected_objects_output)
{
  autoware_msgs::msg::DetectedObjectArray tmp_objects;
  tmp_objects.header = input.header;
  std::vector<size_t> used_targets_indices;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    double tx = targets_[i].x_merge_(0);
    double ty = targets_[i].x_merge_(1);

    double tv = targets_[i].x_merge_(2);
    double tyaw = targets_[i].x_merge_(3);
    double tyaw_rate = targets_[i].x_merge_(4);

    while (tyaw > M_PI)
      tyaw -= 2. * M_PI;
    while (tyaw < -M_PI)
      tyaw += 2. * M_PI;

    tf2::Quaternion q;
    q.setRPY(0, 0, tyaw);

    autoware_msgs::msg::DetectedObject dd;
    dd = targets_[i].object_;
    dd.id = targets_[i].ukf_id_;
    dd.velocity.linear.x = tv;
    dd.acceleration.linear.y = tyaw_rate;
    dd.velocity_reliable = targets_[i].is_stable_;
    dd.pose_reliable = targets_[i].is_stable_;

    if (!targets_[i].is_static_ && targets_[i].is_stable_)
    {
      if (targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y)
      {
        dd.dimensions.x = targets_[i].object_.dimensions.y;
        dd.dimensions.y = targets_[i].object_.dimensions.x;
      }

      dd.pose.position.x = tx;
      dd.pose.position.y = ty;

      if (!std::isnan(q[0]))
        dd.pose.orientation.x = q[0];
      if (!std::isnan(q[1]))
        dd.pose.orientation.y = q[1];
      if (!std::isnan(q[2]))
        dd.pose.orientation.z = q[2];
      if (!std::isnan(q[3]))
        dd.pose.orientation.w = q[3];
    }
    updateBehaviorState(targets_[i], use_sukf_, dd);

    if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init &&
                                   targets_[i].tracking_num_ < TrackingState::Stable))
    {
      tmp_objects.objects.push_back(dd);
      used_targets_indices.push_back(i);
    }
  }
  detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
}

void ImmUkfPda::removeUnnecessaryTarget()
{
  std::vector<UKF> temp_targets;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ != TrackingState::Die)
    {
      temp_targets.push_back(targets_[i]);
    }
  }
  std::vector<UKF>().swap(targets_);
  targets_ = temp_targets;
}

void ImmUkfPda::dumpResultText(autoware_msgs::msg::DetectedObjectArray& detected_objects)
{
  std::ofstream outputfile(result_file_path_, std::ofstream::out | std::ofstream::app);
  for (size_t i = 0; i < detected_objects.objects.size(); i++)
  {
    tf2::Quaternion q(
      detected_objects.objects[i].pose.orientation.x,
      detected_objects.objects[i].pose.orientation.y,
      detected_objects.objects[i].pose.orientation.z,
      detected_objects.objects[i].pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    outputfile << std::to_string(frame_count_) << " " << std::to_string(detected_objects.objects[i].id) << " "
               << "Unknown"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1 -1 -1 -1"
               << " " << std::to_string(detected_objects.objects[i].dimensions.x) << " "
               << std::to_string(detected_objects.objects[i].dimensions.y) << " "
               << "-1"
               << " " << std::to_string(detected_objects.objects[i].pose.position.x) << " "
               << std::to_string(detected_objects.objects[i].pose.position.y) << " "
               << "-1"
               << " " << std::to_string(yaw) << "\n";
  }
  frame_count_++;
}

void ImmUkfPda::tracker(const autoware_msgs::msg::DetectedObjectArray& input,
                        autoware_msgs::msg::DetectedObjectArray& detected_objects_output)
{
  double timestamp = rclcpp::Time(input.header.stamp).seconds();

  if (!init_)
  {
    initTracker(input, timestamp);
    makeOutput(input, std::vector<bool>(), detected_objects_output);
    return;
  }

  double dt = timestamp - timestamp_;
  timestamp_ = timestamp;

  // prevent dt from being too large or negative
  if (dt > 2.0 || dt < 0)
  {
    makeOutput(input, std::vector<bool>(), detected_objects_output);
    return;
  }

  // initialize matching vector
  std::vector<bool> matching_vec(input.objects.size(), false);

  // iterate through existing targets
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }

    // prediction step
    if (use_sukf_)
    {
      targets_[i].predictionSUKF(dt);
    }
    else
    {
      targets_[i].prediction(dt);
    }

    // data association
    std::vector<autoware_msgs::msg::DetectedObject> object_vec;
    bool association_success =
        probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);

    if (!association_success)
    {
      continue;
    }

    // update step
    if (use_sukf_)
    {
      targets_[i].updateSUKF(object_vec);
    }
    else
    {
      targets_[i].update(object_vec);
    }
  }

  // make new targets from unmatched detections
  makeNewTargets(timestamp, input, matching_vec);

  // static classification
  staticClassification();

  // make output
  makeOutput(input, matching_vec, detected_objects_output);

  // remove dead targets
  removeUnnecessaryTarget();
}