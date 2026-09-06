/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
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
 *******************************************************************************/

#include "orbbec_camera/ob_camera_node.h"
#include "libobsensor/hpp/Utils.hpp"
#include <fstream>
#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_NV_HW_DECODER)
#include "orbbec_camera/jetson_nv_decoder.h"
#endif
#include <malloc.h>
#include <fstream>

namespace orbbec_camera {

OBCameraNode::OBCameraNode(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ob::Device> device)
    : node_(node),
      device_(std::move(device)),
      device_info_(device_->getDeviceInfo()),
      rgb_buffer_(nullptr),
      rgb_buffer_left_(nullptr),
      rgb_buffer_right_(nullptr),
      rgb_is_decoded_(false),
      rgb_left_is_decoded_(false),
      rgb_right_is_decoded_(false),
      is_running_(false),
      is_initialized_(false) {
  initializeGlobalImageTransport();
  fps_delay_status_color_ = std::make_unique<FpsDelayStatus>();
  fps_delay_status_depth_ = std::make_unique<FpsDelayStatus>();
  init();
}

void OBCameraNode::init() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  is_running_ = true;
  setupConfig();
  getParameters();
  setupDevices();
  setupDepthPostProcessFilter();
  setupColorPostProcessFilter();
  setupRightIrPostProcessFilter();
  setupLeftIrPostProcessFilter();
  selectBaseStream();
  setupProfiles();
  setupCameraInfo();
  setupTopics();
  setupCameraCtrlServices();
  setupFrameCallback();
  readDefaultExposure();
  readDefaultGain();
  readDefaultWhiteBalance();
#if defined(USE_RK_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<RKMjpegDecoder>(width_[COLOR], height_[COLOR]);
  jpeg_decoder_left_ = std::make_shared<RKMjpegDecoder>(width_[COLOR_LEFT], height_[COLOR_LEFT]);
  jpeg_decoder_right_ = std::make_shared<RKMjpegDecoder>(width_[COLOR_RIGHT], height_[COLOR_RIGHT]);
#elif defined(USE_NV_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
  jpeg_decoder_left_ =
      std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR_LEFT], height_[COLOR_LEFT]);
  jpeg_decoder_right_ =
      std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR_RIGHT], height_[COLOR_RIGHT]);
#endif
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
  if (rgb_buffer_left_) {
    delete[] rgb_buffer_left_;
    rgb_buffer_left_ = nullptr;
  }
  if (rgb_buffer_right_) {
    delete[] rgb_buffer_right_;
    rgb_buffer_right_ = nullptr;
  }

  if (enable_stream_[COLOR]) {
    rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 4];
  } else {
    rgb_buffer_ = nullptr;
  }
  if (enable_stream_[COLOR_LEFT]) {
    rgb_buffer_left_ = new uint8_t[width_[COLOR_LEFT] * height_[COLOR_LEFT] * 4];
  } else {
    rgb_buffer_left_ = nullptr;
  }
  if (enable_stream_[COLOR_RIGHT]) {
    rgb_buffer_right_ = new uint8_t[width_[COLOR_RIGHT] * height_[COLOR_RIGHT] * 4];
  } else {
    rgb_buffer_right_ = nullptr;
  }
  if (enable_colored_point_cloud_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    rgb_point_cloud_buffer_size_ = width_[COLOR] * height_[COLOR] * sizeof(OBColorPoint);
    xy_table_data_size_ = width_[COLOR] * height_[COLOR] * 2;
  }
  rgb_is_decoded_ = false;
  if (diagnostics_frequency_ > 0.0) {
    if (diagnostics_thread_ && diagnostics_thread_->joinable()) {
      diagnostics_thread_->join();
    }
    diagnostics_thread_ = std::make_shared<std::thread>([this]() { setupDiagnosticUpdater(); });
  }
  is_initialized_ = true;
}

bool OBCameraNode::isInitialized() const { return is_initialized_; }

void OBCameraNode::rebootDevice() {
  RCLCPP_INFO(node_->get_logger(), "Do cleanup before rebooting device");
  malloc_trim(0);
  clean();
  malloc_trim(0);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_INFO(node_->get_logger(), "Reboot device");
  if (device_) {
    device_->reboot();
  }
  malloc_trim(0);
  RCLCPP_INFO(node_->get_logger(), "Reboot device DONE");
}

void OBCameraNode::clean() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);

  if (is_cleaned_) {
    return;
  }
  is_cleaned_ = true;

  RCLCPP_INFO(node_->get_logger(), "OBCameraNode::clean() start");
  is_running_ = false;
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }

  if (colorFrameThread_ && colorFrameThread_->joinable()) {
    colorFrameCV_.notify_all();
    colorFrameThread_->join();
  }

  if (leftColorFrameThread_ && leftColorFrameThread_->joinable()) {
    leftColorFrameCV_.notify_all();
    leftColorFrameThread_->join();
  }

  if (rightColorFrameThread_ && rightColorFrameThread_->joinable()) {
    rightColorFrameCV_.notify_all();
    rightColorFrameThread_->join();
  }

  {
    std::unique_lock<std::mutex> lock(colorFrameMtx_);
    while (!colorFrameQueue_.empty()) {
      colorFrameQueue_.pop();
    }
    std::unique_lock<std::mutex> left_lock(leftColorFrameMtx_);
    while (!leftColorFrameQueue_.empty()) {
      leftColorFrameQueue_.pop();
    }
    std::unique_lock<std::mutex> right_lock(rightColorFrameMtx_);
    while (!rightColorFrameQueue_.empty()) {
      rightColorFrameQueue_.pop();
    }
  }
  if (diagnostics_thread_ && diagnostics_thread_->joinable()) {
    diagnostics_thread_->join();
  }

  stopIMU();
  stopStreams();
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
  if (rgb_buffer_left_) {
    delete[] rgb_buffer_left_;
    rgb_buffer_left_ = nullptr;
  }
  if (rgb_buffer_right_) {
    delete[] rgb_buffer_right_;
    rgb_buffer_right_ = nullptr;
  }

  if (color_camera_info_manager_) {
    color_camera_info_manager_.reset();
  }
  if (ir_camera_info_manager_) {
    ir_camera_info_manager_.reset();
  }

  RCLCPP_INFO(node_->get_logger(), "OBCameraNode::clean() end");
}

OBCameraNode::~OBCameraNode() noexcept { clean(); }

void OBCameraNode::getParameters() {
  camera_name_ = node_->declare_parameter<std::string>("camera_name", "camera");
  camera_link_frame_id_ = camera_name_ + "_link";
  for (const auto& stream_index : IMAGE_STREAMS) {
    frame_id_[stream_index] = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    optical_frame_id_[stream_index] =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    width_[stream_index] = node_->declare_parameter<int>(param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    height_[stream_index] = node_->declare_parameter<int>(param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    fps_[stream_index] = node_->declare_parameter<int>(param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    enable_stream_[stream_index] = node_->declare_parameter<bool>(param_name, false);
    param_name = stream_name_[stream_index] + "_flip";
    image_flip_[stream_index] = node_->declare_parameter<bool>(param_name, false);
    param_name = stream_name_[stream_index] + "_mirror";
    image_mirror_[stream_index] = node_->declare_parameter<bool>(param_name, false);
    param_name = stream_name_[stream_index] + "_format";
    format_str_[stream_index] =
        node_->declare_parameter<std::string>(param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    param_name = stream_name_[stream_index] + "_rotation";
    image_rotation_[stream_index] = node_->declare_parameter<int>(param_name, -1);
  }
  depth_aligned_frame_id_[DEPTH] = optical_frame_id_[COLOR];

  publish_tf_ = node_->declare_parameter<bool>("publish_tf", false);
  depth_registration_ = node_->declare_parameter<bool>("depth_registration", false);
  enable_frame_sync_ = node_->declare_parameter<bool>("enable_frame_sync", false);
  enable_pipeline_ = node_->declare_parameter<bool>("enable_pipeline", true);
  enable_point_cloud_ = node_->declare_parameter<bool>("enable_point_cloud", true);
  enable_colored_point_cloud_ = node_->declare_parameter<bool>("enable_colored_point_cloud", false);
  point_cloud_decimation_filter_factor_ =
      node_->declare_parameter<int>("point_cloud_decimation_filter_factor", 1);
  enable_soft_filter_ = node_->declare_parameter<bool>("enable_soft_filter", true);
  enable_color_auto_exposure_ = node_->declare_parameter<bool>("enable_color_auto_exposure", true);
  enable_ir_auto_exposure_ = node_->declare_parameter<bool>("enable_ir_auto_exposure", true);
  enable_d2c_viewer_ = node_->declare_parameter<bool>("enable_d2c_viewer", false);
  align_mode_ = node_->declare_parameter<std::string>("align_mode", "HW");
  std::string align_target_stream_str_ = node_->declare_parameter<std::string>("align_target_stream", "COLOR");
  align_target_stream_ = obStreamTypeFromString(align_target_stream_str_);
  enable_depth_scale_ = node_->declare_parameter<bool>("enable_depth_scale", true);
  ordered_pc_ = node_->declare_parameter<bool>("ordered_pc", false);
  time_domain_ = node_->declare_parameter<std::string>("time_domain", "global");
  tf_publish_rate_ = node_->declare_parameter<double>("tf_publish_rate", 0.0);
  diagnostics_frequency_ = node_->declare_parameter<double>("diagnostics_frequency", 1.0);

  if (enable_colored_point_cloud_ || enable_d2c_viewer_) {
    depth_registration_ = true;
  }
}

void OBCameraNode::startStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    if (enable_frame_sync_) {
      RCLCPP_INFO(node_->get_logger(), "====Enable frame sync====");
      pipeline_->enableFrameSync();
    } else {
      pipeline_->disableFrameSync();
    }
    try {
      setupPipelineConfig();
      pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
        this->onNewFrameSetCallback(frame_set);
      });
    } catch (const ob::Error& e) {
      RCLCPP_ERROR(node_->get_logger(), "failed to start pipeline: %s", e.getMessage());
      enable_stream_[INFRA0] = false;
      setupPipelineConfig();
      pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
        this->onNewFrameSetCallback(frame_set);
      });
    }
    if (!colorFrameThread_ && enable_stream_[COLOR]) {
      RCLCPP_INFO(node_->get_logger(), "Create color frame read thread.");
      colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
    }
    if (!leftColorFrameThread_ && enable_stream_[COLOR_LEFT]) {
      RCLCPP_INFO(node_->get_logger(), "Create left color frame read thread.");
      leftColorFrameThread_ =
          std::make_shared<std::thread>([this]() { onNewLeftColorFrameCallback(); });
    }
    if (!rightColorFrameThread_ && enable_stream_[COLOR_RIGHT]) {
      RCLCPP_INFO(node_->get_logger(), "Create right color frame read thread.");
      rightColorFrameThread_ =
          std::make_shared<std::thread>([this]() { onNewRightColorFrameCallback(); });
    }
    pipeline_started_ = true;
  } else {
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (enable_stream_[stream_index] && !stream_started_[stream_index]) {
        startStream(stream_index);
      }
    }
  }
}

void OBCameraNode::stopStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_ && pipeline_ && pipeline_started_) {
    try {
      pipeline_->stop();
    } catch (const ob::Error& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to stop pipeline: %s", e.getMessage());
    }
    pipeline_started_ = false;
  } else {
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (stream_started_[stream_index]) {
        stopStream(stream_index);
      }
    }
  }
}

void OBCameraNode::stopIMU() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_sync_output_accel_gyro_) {
    if (!imu_sync_output_start_ || !imuPipeline_) {
      return;
    }
    try {
      imuPipeline_->stop();
    } catch (const ob::Error& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to stop imu pipeline: %s", e.getMessage());
    }
  } else {
    for (const auto& stream_index : HID_STREAMS) {
      if (imu_started_[stream_index]) {
        imu_sensor_[stream_index]->stop();
        imu_started_[stream_index] = false;
      }
    }
  }
}

void OBCameraNode::startStream(const stream_index_pair& stream_index) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot start stream when pipeline is enabled");
    return;
  }
  if (!enable_stream_[stream_index]) {
    return;
  }
  if (stream_started_[stream_index]) {
    return;
  }
  auto callback = frame_callback_[stream_index];
  auto profile = stream_profile_[stream_index];
  try {
    sensors_[stream_index]->startStream(profile, callback);
    stream_started_[stream_index] = true;
    if (!colorFrameThread_ && stream_index == COLOR) {
      colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
    }
    if (!leftColorFrameThread_ && stream_index == COLOR_LEFT) {
      leftColorFrameThread_ =
          std::make_shared<std::thread>([this]() { onNewLeftColorFrameCallback(); });
    }
    if (!rightColorFrameThread_ && stream_index == COLOR_RIGHT) {
      rightColorFrameThread_ =
          std::make_shared<std::thread>([this]() { onNewRightColorFrameCallback(); });
    }
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start stream %s.",
                 stream_name_[stream_index].c_str());
  }
}

void OBCameraNode::stopStream(const stream_index_pair& stream_index) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    return;
  }
  if (!stream_started_[stream_index]) {
    return;
  }
  sensors_[stream_index]->stopStream();
  stream_started_[stream_index] = false;
}

void OBCameraNode::publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  try {
    if (depth_registration_ || enable_colored_point_cloud_) {
      if (frame_set->depthFrame() != nullptr && frame_set->colorFrame() != nullptr) {
        publishColoredPointCloud(frame_set);
      }
    }
    if (enable_point_cloud_ && frame_set->depthFrame() != nullptr) {
      publishDepthPointCloud(frame_set);
    }
  } catch (const ob::Error& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.getMessage());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "publishPointCloud with unknown error");
  }
}

void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!depth_cloud_pub_ || depth_cloud_pub_->get_subscription_count() == 0 || !enable_point_cloud_) {
    return;
  }
  std::lock_guard<decltype(cloud_mutex_)> cloud_lock(cloud_mutex_);
  auto depth_frame = frame_set->depthFrame();
  if (!depth_frame) {
    RCLCPP_ERROR(node_->get_logger(), "depth frame is null");
    return;
  }
  auto camera_params = pipeline_->getCameraParam();
  if (depth_registration_ && isGemini335PID(device_info_->pid())) {
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  depth_point_cloud_filter_.setCameraParam(camera_params);
  float depth_scale = depth_frame->getValueScale();
  depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
  depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  depth_point_cloud_filter_.setDecimationFactor(point_cloud_decimation_filter_factor_);
  auto result_frame = depth_point_cloud_filter_.process(depth_frame);
  if (!result_frame) {
    return;
  }
  auto point_size = result_frame->dataSize() / sizeof(OBPoint);
  auto* points = reinterpret_cast<OBPoint*>(result_frame->data());

  sensor_msgs::msg::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  auto width = depth_frame->width();
  auto height = depth_frame->height();
  modifier.resize(width * height);
  cloud_msg.width = width;
  cloud_msg.height = height;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  size_t valid_count = 0;
  for (size_t i = 0; i < point_size; i++) {
    bool valid_point = points[i].z >= 20.0 && points[i].z <= 10000.0;
    if (valid_point || ordered_pc_) {
      *iter_x = static_cast<float>(points[i].x / 1000.0);
      *iter_y = static_cast<float>(points[i].y / 1000.0);
      *iter_z = static_cast<float>(points[i].z / 1000.0);
      ++iter_x; ++iter_y; ++iter_z;
      valid_count++;
    }
  }
  if (!ordered_pc_) {
    cloud_msg.is_dense = true;
    cloud_msg.width = valid_count;
    cloud_msg.height = 1;
    modifier.resize(valid_count);
  }
  auto frame_timestamp = getFrameTimestampUs(depth_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  std::string frame_id = depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  cloud_msg.header.stamp = timestamp;
  cloud_msg.header.frame_id = frame_id;
  depth_cloud_pub_->publish(cloud_msg);
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!depth_registered_cloud_pub_ || depth_registered_cloud_pub_->get_subscription_count() == 0 ||
      !enable_colored_point_cloud_) {
    return;
  }
  std::lock_guard<decltype(cloud_mutex_)> cloud_lock(cloud_mutex_);
  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();
  if (!depth_frame || !color_frame) {
    return;
  }
  auto camera_params = pipeline_->getCameraParam();
  if (depth_registration_) {
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  color_point_cloud_filter_.setCameraParam(camera_params);
  auto depth_scale = depth_frame->getValueScale();
  color_point_cloud_filter_.setPositionDataScaled(depth_scale);
  color_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_RGB_POINT);
  color_point_cloud_filter_.setDecimationFactor(point_cloud_decimation_filter_factor_);
  auto result_frame = color_point_cloud_filter_.process(frame_set);
  if (!result_frame) {
    return;
  }
  auto point_size = result_frame->dataSize() / sizeof(OBColorPoint);
  auto* point_cloud = static_cast<OBColorPoint*>(result_frame->data());

  sensor_msgs::msg::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  cloud_msg.width = color_frame->width();
  cloud_msg.height = color_frame->height();
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  size_t valid_count = 0;
  for (size_t i = 0; i < point_size; i++) {
    bool valid_point = point_cloud[i].z >= 20.0 && point_cloud[i].z <= 10000.0;
    if (valid_point || ordered_pc_) {
      *iter_x = static_cast<float>(point_cloud[i].x / 1000.0);
      *iter_y = static_cast<float>(point_cloud[i].y / 1000.0);
      *iter_z = static_cast<float>(point_cloud[i].z / 1000.0);
      ++iter_x; ++iter_y; ++iter_z;
      ++valid_count;
    }
  }
  if (!ordered_pc_) {
    cloud_msg.is_dense = true;
    cloud_msg.width = valid_count;
    cloud_msg.height = 1;
    modifier.resize(valid_count);
  }
  auto frame_timestamp = getFrameTimestampUs(depth_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  cloud_msg.header.stamp = timestamp;
  cloud_msg.header.frame_id = optical_frame_id_[COLOR];
  depth_registered_cloud_pub_->publish(cloud_msg);
}

void OBCameraNode::setDefaultIMUMessage(sensor_msgs::msg::Imu& imu_msg) {
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {
      linear_accel_cov_, 0.0, 0.0, 0.0, linear_accel_cov_, 0.0, 0.0, 0.0, linear_accel_cov_};
  imu_msg.angular_velocity_covariance = {
      angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_};
}

void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                                         const stream_index_pair& stream_index) {
  if (!isInitialized()) {
    return;
  }
  if (!imu_publishers_.count(stream_index)) {
    return;
  }
  auto has_subscriber = imu_publishers_[stream_index]->get_subscription_count() > 0;
  if (!has_subscriber) {
    return;
  }

  auto imu_msg = sensor_msgs::msg::Imu();
  setDefaultIMUMessage(imu_msg);
  imu_msg.header.frame_id = optical_frame_id_[stream_index];
  auto frame_timestamp = getFrameTimestampUs(frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  imu_msg.header.stamp = timestamp;

  if (frame->type() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->value();
    imu_msg.angular_velocity.x = data.x;
    imu_msg.angular_velocity.y = data.y;
    imu_msg.angular_velocity.z = data.z;
  } else if (frame->type() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->value();
    imu_msg.linear_acceleration.x = data.x;
    imu_msg.linear_acceleration.y = data.y;
    imu_msg.linear_acceleration.z = data.z;
  }
  imu_publishers_[stream_index]->publish(imu_msg);
}

bool OBCameraNode::decodeColorFrameToBuffer(const std::shared_ptr<ob::Frame>& frame,
                                            uint8_t* dest) {
  if (!rgb_buffer_ && !rgb_buffer_left_ && !rgb_buffer_right_) {
    return false;
  }
  stream_index_pair stream_index = COLOR;
  switch (frame->getType()) {
    case OB_FRAME_COLOR:
      stream_index = COLOR;
      break;
    case OB_FRAME_COLOR_LEFT:
      stream_index = COLOR_LEFT;
      break;
    case OB_FRAME_COLOR_RIGHT:
      stream_index = COLOR_RIGHT;
      break;
    default:
      stream_index = COLOR;
      break;
  }
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (enable_colored_point_cloud_ && depth_registered_cloud_pub_->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (metadata_publishers_.count(stream_index) &&
      metadata_publishers_[stream_index]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (camera_info_publishers_.count(stream_index) &&
      camera_info_publishers_[stream_index]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return false;
  }
  bool is_decoded = false;
  if (!frame) {
    return false;
  }
#if defined(USE_RK_HW_DECODER) || defined(USE_NV_HW_DECODER)
  std::shared_ptr<JPEGDecoder> decoder;
  if (stream_index == COLOR_LEFT) {
    decoder = jpeg_decoder_left_;
  } else if (stream_index == COLOR_RIGHT) {
    decoder = jpeg_decoder_right_;
  } else {
    decoder = mjpeg_decoder_;
  }
  if (frame && frame->format() != OB_FORMAT_RGB888) {
    if (frame->format() == OB_FORMAT_MJPG && decoder) {
      auto video_frame = frame->as<ob::ColorFrame>();
      bool ret = decoder->decode(video_frame, dest);
      if (!ret) {
        RCLCPP_ERROR(node_->get_logger(), "Decode frame failed");
        is_decoded = false;
      } else {
        is_decoded = true;
      }
    }
  }
#endif
  if (!is_decoded) {
    auto video_frame = softwareDecodeColorFrame(frame);
    if (!video_frame) {
      RCLCPP_ERROR(node_->get_logger(), "Decode frame failed");
      return false;
    }
    memcpy(dest, video_frame->data(), video_frame->dataSize());
    return true;
  }
  return true;
}

std::shared_ptr<ob::Frame> OBCameraNode::decodeIRMJPGFrame(
    const std::shared_ptr<ob::Frame>& frame) {
  if (frame->format() == OB_FORMAT_MJPEG &&
      (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
       frame->type() == OB_FRAME_IR_RIGHT)) {
    auto video_frame = frame->as<ob::IRFrame>();
    cv::Mat mjpgMat(1, video_frame->dataSize(), CV_8UC1, video_frame->data());
    cv::Mat irRawMat = cv::imdecode(mjpgMat, cv::IMREAD_GRAYSCALE);
    std::shared_ptr<ob::Frame> irFrame = ob::FrameHelper::createFrame(
        video_frame->type(), video_frame->format(), video_frame->dataSize());
    uint32_t buffer_size = irRawMat.rows * irRawMat.cols * irRawMat.channels();
    if (buffer_size > irFrame->dataSize()) {
      RCLCPP_ERROR(node_->get_logger(), "Insufficient buffer size allocation");
      return nullptr;
    }
    memcpy(irFrame->data(), irRawMat.data, buffer_size);
    ob::FrameHelper::setFrameDeviceTimestamp(irFrame, video_frame->timeStamp());
    ob::FrameHelper::setFrameDeviceTimestampUs(irFrame, video_frame->timeStampUs());
    ob::FrameHelper::setFrameSystemTimestamp(irFrame, video_frame->systemTimeStamp());
    return irFrame;
  }
  return nullptr;
}

std::shared_ptr<ob::Frame> OBCameraNode::processRightIrFrameFilter(
    std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr || frame->getType() != OB_FRAME_IR_RIGHT) {
    return nullptr;
  }
  for (size_t i = 0; i < right_ir_filter_list_.size(); i++) {
    auto filter = right_ir_filter_list_[i];
    if (filter->isEnabled() && frame != nullptr) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "Right Ir filter process failed");
        break;
      }
    }
  }
  return frame;
}

std::shared_ptr<ob::Frame> OBCameraNode::processLeftIrFrameFilter(
    std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr || frame->getType() != OB_FRAME_IR_LEFT) {
    return nullptr;
  }
  for (size_t i = 0; i < left_ir_filter_list_.size(); i++) {
    auto filter = left_ir_filter_list_[i];
    if (filter->isEnabled() && frame != nullptr) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "Left Ir filter process failed");
        break;
      }
    }
  }
  return frame;
}

std::shared_ptr<ob::Frame> OBCameraNode::processColorFrameFilter(
    std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  auto frame_type = frame->type();
  if (frame_type == OB_FRAME_COLOR) {
    for (size_t i = 0; i < color_filter_list_.size(); i++) {
      auto filter = color_filter_list_[i];
      if (filter->isEnabled() && frame != nullptr) {
        frame = filter->process(frame);
        if (frame == nullptr) {
          RCLCPP_ERROR(node_->get_logger(), "Color filter process failed");
          break;
        }
      }
    }
    return frame;
  } else if (frame_type == OB_FRAME_COLOR_LEFT) {
    for (size_t i = 0; i < left_color_filter_list_.size(); i++) {
      auto filter = left_color_filter_list_[i];
      if (filter->isEnabled() && frame != nullptr) {
        frame = filter->process(frame);
        if (frame == nullptr) {
          RCLCPP_ERROR(node_->get_logger(), "Left Color filter process failed");
          break;
        }
      }
    }
    return frame;
  } else if (frame_type == OB_FRAME_COLOR_RIGHT) {
    for (size_t i = 0; i < right_color_filter_list_.size(); i++) {
      auto filter = right_color_filter_list_[i];
      if (filter->isEnabled() && frame != nullptr) {
        frame = filter->process(frame);
        if (frame == nullptr) {
          RCLCPP_ERROR(node_->get_logger(), "Right Color filter process failed");
          break;
        }
      }
    }
    return frame;
  }
  return frame;
}

std::shared_ptr<ob::Frame> OBCameraNode::processDepthFrameFilter(
    std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr || frame->type() != OB_FRAME_DEPTH) {
    return nullptr;
  }
  for (size_t i = 0; i < depth_filter_list_.size(); i++) {
    auto filter = depth_filter_list_[i];
    if (filter->isEnabled() && frame != nullptr) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "Depth filter process failed");
        break;
      }
    }
  }
  return frame;
}

uint64_t OBCameraNode::getFrameTimestampUs(const std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr) {
    return 0;
  }
  if (time_domain_ == "device") {
    return frame->timeStampUs();
  } else if (time_domain_ == "global") {
    return frame->globalTimeStampUs();
  } else {
    return frame->systemTimeStampUs();
  }
}

void OBCameraNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
  if (!is_running_) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Frame callback called before initialization");
    return;
  }
  if (!isInitialized()) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Frame callback called before initialization");
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  RCLCPP_INFO_ONCE(node_->get_logger(), "Received first frame set");
  try {
    // Retrieve available frames from the frameset
    auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
    auto left_color_frame = frame_set->getFrame(OB_FRAME_COLOR_LEFT);
    auto right_color_frame = frame_set->getFrame(OB_FRAME_COLOR_RIGHT);
    auto left_ir_frame = frame_set->getFrame(OB_FRAME_IR_LEFT);
    auto right_ir_frame = frame_set->getFrame(OB_FRAME_IR_RIGHT);

    // Apply per-stream processing filters and update the frameset as needed
    if (depth_frame) {
      depth_frame = processDepthFrameFilter(depth_frame);
      if (depth_frame) {
        frame_set->pushFrame(depth_frame);
      }
    }
    if (color_frame) {
      color_frame = processColorFrameFilter(color_frame);
      if (color_frame) {
        frame_set->pushFrame(color_frame);
      }
    }
    if (left_color_frame) {
      left_color_frame = processColorFrameFilter(left_color_frame);
      if (left_color_frame) {
        frame_set->pushFrame(left_color_frame);
      }
    }
    if (right_color_frame) {
      right_color_frame = processColorFrameFilter(right_color_frame);
      if (right_color_frame) {
        frame_set->pushFrame(right_color_frame);
      }
    }
    if (left_ir_frame) {
      left_ir_frame = processLeftIrFrameFilter(left_ir_frame);
      if (left_ir_frame) {
        frame_set->pushFrame(left_ir_frame);
      }
    }
    if (right_ir_frame) {
      right_ir_frame = processRightIrFrameFilter(right_ir_frame);
      if (right_ir_frame) {
        frame_set->pushFrame(right_ir_frame);
      }
    }

    // Perform alignment when depth registration is enabled
    if (depth_registration_ && align_filter_ && depth_frame && color_frame) {
      if (align_mode_ == "SW" || enable_d2c_viewer_) {
        auto aligned_frame_set = align_filter_->process(frame_set);
        if (aligned_frame_set) {
          frame_set = aligned_frame_set->as<ob::FrameSet>();
        }
      }
    }

    // Dispatch color frames via internal queues/threads
    if (enable_stream_[COLOR] && color_frame) {
      {
        std::unique_lock<std::mutex> lock(colorFrameMtx_);
        colorFrameQueue_.push(frame_set);
      }
      colorFrameCV_.notify_one();
    } else if (enable_stream_[COLOR_LEFT] && left_color_frame) {
      {
        std::unique_lock<std::mutex> lock(leftColorFrameMtx_);
        leftColorFrameQueue_.push(frame_set);
      }
      leftColorFrameCV_.notify_one();
    } else if (enable_stream_[COLOR_RIGHT] && right_color_frame) {
      {
        std::unique_lock<std::mutex> lock(rightColorFrameMtx_);
        rightColorFrameQueue_.push(frame_set);
      }
      rightColorFrameCV_.notify_one();
    } else {
      // No color stream active, publish point cloud directly
      publishPointCloud(frame_set);
    }

    // Forward non-color streams through the common single-frame handler
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (stream_index == COLOR || stream_index == COLOR_LEFT || stream_index == COLOR_RIGHT) {
        continue;
      }
      auto frame = frame_set->getFrame(stream_index.first);
      if (frame == nullptr) {
        continue;
      }
      // Attempt to decode IR MJPG frames
      auto decoded_ir = decodeIRMJPGFrame(frame);
      if (decoded_ir) {
        frame = decoded_ir;
      }
      onNewFrameCallback(frame, stream_index);
    }

  } catch (const ob::Error& e) {
    RCLCPP_ERROR(node_->get_logger(), "onNewFrameSetCallback error: %s", e.getMessage());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "onNewFrameSetCallback error: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "onNewFrameSetCallback error: unknown error");
  }
}

void OBCameraNode::onNewColorFrameCallback() {
  while (enable_stream_[COLOR] && rclcpp::ok() && is_running_.load()) {
    std::unique_lock<std::mutex> lock(colorFrameMtx_);
    colorFrameCV_.wait(lock, [this]() {
      return !colorFrameQueue_.empty() || !(is_running_.load());
    });

    if (!is_running_.load()) {
      break;
    }

    if (colorFrameQueue_.empty()) {
      continue;
    }

    auto frame_set = colorFrameQueue_.front();
    colorFrameQueue_.pop();
    lock.unlock();

    if (frame_set == nullptr) {
      continue;
    }

    auto color_frame = frame_set->colorFrame();
    if (color_frame == nullptr) {
      continue;
    }

    // Decode color frame into RGB buffer
    rgb_is_decoded_ = decodeColorFrameToBuffer(color_frame, rgb_buffer_);

    // Publish point cloud with the decoded color data
    publishPointCloud(frame_set);

    // Forward the color frame to the common single-frame handler
    onNewFrameCallback(color_frame, COLOR);
  }

  RCLCPP_INFO(node_->get_logger(), "Color frame thread exit!");
}

void OBCameraNode::onNewLeftColorFrameCallback() {
  while (enable_stream_[COLOR_LEFT] && rclcpp::ok() && is_running_.load()) {
    std::unique_lock<std::mutex> lock(leftColorFrameMtx_);
    leftColorFrameCV_.wait(lock, [this]() {
      return !leftColorFrameQueue_.empty() || !(is_running_.load());
    });

    if (!is_running_.load()) {
      break;
    }

    if (leftColorFrameQueue_.empty()) {
      continue;
    }

    auto frame_set = leftColorFrameQueue_.front();
    leftColorFrameQueue_.pop();
    lock.unlock();

    if (frame_set == nullptr) {
      continue;
    }

    auto left_color_frame = frame_set->getFrame(OB_FRAME_COLOR_LEFT);
    if (left_color_frame == nullptr) {
      continue;
    }

    // Decode into the left buffer when needed
    rgb_left_is_decoded_ = decodeColorFrameToBuffer(left_color_frame, rgb_buffer_left_);

    // Forward to the common handler
    onNewFrameCallback(left_color_frame, COLOR_LEFT);
  }

  RCLCPP_INFO(node_->get_logger(), "Left Color frame thread exit!");
}

void OBCameraNode::onNewRightColorFrameCallback() {
  while (enable_stream_[COLOR_RIGHT] && rclcpp::ok() && is_running_.load()) {
    std::unique_lock<std::mutex> lock(rightColorFrameMtx_);
    rightColorFrameCV_.wait(lock, [this]() {
      return !rightColorFrameQueue_.empty() || !(is_running_.load());
    });

    if (!is_running_.load()) {
      break;
    }

    if (rightColorFrameQueue_.empty()) {
      continue;
    }

    auto frame_set = rightColorFrameQueue_.front();
    rightColorFrameQueue_.pop();
    lock.unlock();

    if (frame_set == nullptr) {
      continue;
    }

    auto right_color_frame = frame_set->getFrame(OB_FRAME_COLOR_RIGHT);
    if (right_color_frame == nullptr) {
      continue;
    }

    // Decode into the right buffer when required
    rgb_right_is_decoded_ = decodeColorFrameToBuffer(right_color_frame, rgb_buffer_right_);

    // Forward to the common handler
    onNewFrameCallback(right_color_frame, COLOR_RIGHT);
  }

  RCLCPP_INFO(node_->get_logger(), "Right Color frame thread exit!");
}

std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame>& frame) {
  if (frame->format() == OB_FORMAT_RGB || frame->format() == OB_FORMAT_BGR) {
    return frame;
  }
  if (frame->format() == OB_FORMAT_Y16 || frame->format() == OB_FORMAT_Y8) {
    return frame;
  }
  if (frame->format() == OB_FORMAT_RGBA || frame->format() == OB_FORMAT_BGRA) {
    return frame;
  }
  if (!setupFormatConvertType(frame->format())) {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported color format: %d", frame->format());
    return nullptr;
  }
  auto covert_frame = format_convert_filter_.process(frame);
  if (covert_frame == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Format convert to RGB888 failed");
    return nullptr;
  }
  return covert_frame;
}

void OBCameraNode::onNewFrameCallback(std::shared_ptr<ob::Frame> frame,
                                      const stream_index_pair& stream_index) {
  if (frame == nullptr) {
    return;
  }

  // Determine whether this frame should be processed/published
  // Consider subscriptions for image, camera_info, and metadata
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (camera_info_publishers_.count(stream_index) &&
      camera_info_publishers_[stream_index]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (metadata_publishers_.count(stream_index) &&
      metadata_publishers_[stream_index]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return;
  }

  // Convert the raw frame into the appropriate typed video frame
  auto video_frame = frame->as<ob::VideoFrame>();
  if (!video_frame) {
    return;
  }

  // Derive width/height, timestamp, and the correct frame_id
  int width = video_frame->width();
  int height = video_frame->height();
  auto frame_timestamp = getFrameTimestampUs(frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);

  std::string frame_id = (depth_registration_ && stream_index == DEPTH)
                             ? depth_aligned_frame_id_[stream_index]
                             : optical_frame_id_[stream_index];

  if (color_camera_info_manager_ && color_camera_info_manager_->isCalibrated() &&
      stream_index == COLOR) {
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    auto camera_info = color_camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher->publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else if (ir_camera_info_manager_ && ir_camera_info_manager_->isCalibrated() &&
             (stream_index == INFRA0 || stream_index == DEPTH)) {
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    auto camera_info = ir_camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher->publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else {
    OBCameraIntrinsic intrinsic;
    OBCameraDistortion distortion;
    if (isGemini335PID(device_info_->pid())) {
      auto stream_profile = frame->getStreamProfile();
      auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
      intrinsic = video_stream_profile->getIntrinsic();
      distortion = video_stream_profile->getDistortion();
    } else {
      auto camera_params = pipeline_->getCameraParam();
      intrinsic = stream_index == COLOR ? camera_params.rgbIntrinsic : camera_params.depthIntrinsic;
      distortion =
          stream_index == COLOR ? camera_params.rgbDistortion : camera_params.depthDistortion;
    }
    auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher->publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  }

  if (!image_publishers_[stream_index].getNumSubscribers()) {
    return;
  }

  // Populate and publish the image message
  auto& image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image = cv::Mat(height, width, CV_16UC1);
  }

  // Handle decoded color buffers
  bool is_color_stream = (stream_index == COLOR || stream_index == COLOR_LEFT ||
                          stream_index == COLOR_RIGHT);
  uint8_t* decoded_buffer = nullptr;
  bool is_decoded = false;
  if (stream_index == COLOR) {
    decoded_buffer = rgb_buffer_;
    is_decoded = rgb_is_decoded_;
  } else if (stream_index == COLOR_LEFT) {
    decoded_buffer = rgb_buffer_left_;
    is_decoded = rgb_left_is_decoded_;
  } else if (stream_index == COLOR_RIGHT) {
    decoded_buffer = rgb_buffer_right_;
    is_decoded = rgb_right_is_decoded_;
  }

  std::string encoding;
  if (is_color_stream && is_decoded && decoded_buffer) {
    image = cv::Mat(height, width, CV_8UC3, decoded_buffer);
    encoding = sensor_msgs::image_encodings::RGB8;
  } else {
    auto data = video_frame->data();
    auto data_size = video_frame->dataSize();
    if (video_frame->format() == OB_FORMAT_Y16 || video_frame->format() == OB_FORMAT_Z16) {
      image = cv::Mat(height, width, CV_16UC1, data);
      encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    } else if (video_frame->format() == OB_FORMAT_Y8) {
      image = cv::Mat(height, width, CV_8UC1, data);
      encoding = sensor_msgs::image_encodings::MONO8;
    } else if (video_frame->format() == OB_FORMAT_RGB888 || video_frame->format() == OB_FORMAT_RGB) {
      image = cv::Mat(height, width, CV_8UC3, data);
      encoding = sensor_msgs::image_encodings::RGB8;
    } else if (video_frame->format() == OB_FORMAT_BGR) {
      image = cv::Mat(height, width, CV_8UC3, data);
      encoding = sensor_msgs::image_encodings::BGR8;
    } else if (video_frame->format() == OB_FORMAT_RGBA) {
      image = cv::Mat(height, width, CV_8UC4, data);
      encoding = sensor_msgs::image_encodings::RGBA8;
    } else if (video_frame->format() == OB_FORMAT_BGRA) {
      image = cv::Mat(height, width, CV_8UC4, data);
      encoding = sensor_msgs::image_encodings::BGRA8;
    } else {
      image = cv::Mat(height, width, CV_8UC1, data);
      encoding = sensor_msgs::image_encodings::MONO8;
    }
  }

  // Apply depth scaling for DEPTH stream
  if (stream_index == DEPTH && enable_depth_scale_) {
    auto depth_video_frame = frame->as<ob::DepthFrame>();
    if (depth_video_frame) {
      float depth_scale = depth_video_frame->getValueScale();
      if (std::abs(depth_scale - 1.0f) > 1e-6) {
        image.convertTo(image, CV_16UC1, depth_scale);
      }
    }
  }

  // Publish either raw or flipped output
  sensor_msgs::msg::Image::SharedPtr image_msg;
  if (image_flip_[stream_index]) {
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 1);
    auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, flipped_image);
    image_msg = cv_img.toImageMsg();
  } else {
    auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image);
    image_msg = cv_img.toImageMsg();
  }

  image_msg->header.stamp = timestamp;
  image_msg->header.frame_id = frame_id;
  image_publishers_[stream_index].publish(image_msg);

  saveImageToFile(stream_index, image, image_msg);
}

void OBCameraNode::publishMetadata(const std::shared_ptr<ob::Frame>& frame,
                                   const stream_index_pair& stream_index,
                                   const std_msgs::msg::Header& header) {
  if (metadata_publishers_.count(stream_index) == 0) {
    return;
  }
  auto metadata_publisher = metadata_publishers_[stream_index];
  if (metadata_publisher->get_subscription_count() == 0) {
    return;
  }
  orbbec_camera::msg::Metadata metadata_msg;
  metadata_msg.header = header;
  nlohmann::json json_data;
  for (int i = 0; i < OB_FRAME_METADATA_TYPE_COUNT; i++) {
    auto meta_data_type = static_cast<OBFrameMetadataType>(i);
    std::string field_name = metaDataTypeToString(meta_data_type);
    if (!frame->hasMetadata(meta_data_type)) {
      continue;
    }
    int64_t value = frame->getMetadataValue(meta_data_type);
    json_data[field_name] = value;
  }
  metadata_msg.json_data = json_data.dump(2);
  metadata_publisher->publish(metadata_msg);
}

void OBCameraNode::saveImageToFile(const stream_index_pair& stream_index, const cv::Mat& image,
                                   const sensor_msgs::msg::Image::SharedPtr& image_msg) {
  if (save_images_[stream_index]) {
    auto now = time(nullptr);
    std::stringstream ss;
    ss << std::put_time(localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = std::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    int index = save_images_count_[stream_index];
    std::string file_suffix = stream_index == COLOR ? ".png" : ".raw";
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg->width) + "x" +
                           std::to_string(image_msg->height) + "_" + std::to_string(fps) + "hz_" +
                           ss.str() + "_" + std::to_string(index) + file_suffix;
    if (!std::filesystem::exists(current_path + "/image")) {
      std::filesystem::create_directory(current_path + "/image");
    }
    RCLCPP_INFO(node_->get_logger(), "Saving image to %s", filename.c_str());
    if (stream_index.first == OB_STREAM_COLOR) {
      auto image_to_save =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imwrite(filename, image_to_save);
    } else {
      std::ofstream ofs(filename, std::ios::out | std::ios::binary);
      if (ofs.is_open()) {
        if (image.isContinuous()) {
          ofs.write(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
        } else {
          int rows = image.rows;
          int cols = image.cols * image.channels();
          for (int r = 0; r < rows; ++r) {
            ofs.write(reinterpret_cast<const char*>(image.ptr<uchar>(r)), cols);
          }
        }
        ofs.close();
      }
    }
    if (++save_images_count_[stream_index] >= max_save_images_count_) {
      save_images_[stream_index] = false;
    }
  }
}

void OBCameraNode::imageSubscribedCallback(const stream_index_pair& stream_index) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (!device_ || !device_info_ || !is_initialized_ || !is_running_.load()) {
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Image stream %s subscribed",
              stream_name_[stream_index].c_str());
  if (enable_pipeline_) {
    if (pipeline_started_) {
      return;
    }
    startStreams();
  } else {
    if (stream_started_[stream_index]) {
      return;
    }
    startStream(stream_index);
  }
}

void OBCameraNode::imageUnsubscribedCallback(const stream_index_pair& stream_index) {
  RCLCPP_INFO(node_->get_logger(), "Image stream %s unsubscribed",
              stream_name_[stream_index].c_str());
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    if (!pipeline_started_) {
      return;
    }
    bool all_stream_no_subscriber = true;
    for (auto& item : image_publishers_) {
      if (item.second.getNumSubscribers() > 0) {
        all_stream_no_subscriber = false;
        break;
      }
    }
    if (all_stream_no_subscriber) {
      stopStreams();
    }
  } else {
    if (!stream_started_[stream_index]) {
      return;
    }
    auto subscriber_count = image_publishers_[stream_index].getNumSubscribers();
    if (subscriber_count == 0) {
      stopStream(stream_index);
    }
  }
}

void OBCameraNode::publishStaticTF(const rclcpp::Time& t, const tf2::Vector3& trans,
                                   const tf2::Quaternion& q, const std::string& from,
                                   const std::string& to) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans[2] / 1000.0;
  msg.transform.translation.y = -trans[0] / 1000.0;
  msg.transform.translation.z = -trans[1] / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);
  if (!stream_profile_.count(base_stream_)) {
    RCLCPP_ERROR(node_->get_logger(), "Base stream is not available");
    return;
  }
  auto base_stream_profile = stream_profile_[base_stream_];
  for (const auto& item : stream_profile_) {
    auto stream_index = item.first;
    auto stream_profile = item.second;
    if (!stream_profile) {
      continue;
    }
    OBExtrinsic ex;
    try {
      ex = stream_profile->getExtrinsicTo(base_stream_profile);
    } catch (const ob::Error& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get %s extrinsic: %s",
                   stream_name_[stream_index].c_str(), e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    auto Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    Q = Q.normalize();
    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);
    auto timestamp = node_->now();
    if (stream_index.first != base_stream_.first) {
      publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
    }
    publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
  }
}

void OBCameraNode::publishDynamicTransforms() {
  RCLCPP_WARN(node_->get_logger(), "Publishing dynamic camera transforms (/tf) at %g Hz",
              tf_publish_rate_);
  static std::mutex mu;
  std::unique_lock<std::mutex> lock(mu);
  while (rclcpp::ok() && is_running_) {
    tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                    [this] { return (!(is_running_)); });
    {
      auto t = node_->now();
      for (auto& msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
}

void OBCameraNode::publishStaticTransforms() {
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

bool OBCameraNode::isGemini335PID(uint32_t pid) {
  const uint16_t GEMINI_335_PID = 0x0800;
  const uint16_t GEMINI_330_PID = 0x0801;
  const uint16_t GEMINI_336_PID = 0x0803;
  const uint16_t GEMINI_335L_PID = 0x0804;
  const uint16_t GEMINI_330L_PID = 0x0805;
  const uint16_t GEMINI_336L_PID = 0x0807;
  const uint16_t GEMINI_335LG_PID = 0x080B;
  const uint16_t GEMINI_336LG_PID = 0x080D;
  const uint16_t GEMINI_335LE_PID = 0x080E;
  const uint16_t GEMINI_336LE_PID = 0x0810;
  const int32_t CUSTOM_ADVANTECH_GEMINI_336_PID = 0x0816;
  const int32_t CUSTOM_ADVANTECH_GEMINI_336L_PID = 0x0817;
  const uint16_t GEMINI_338_PID = 0x0818;
  return pid == GEMINI_335_PID || pid == GEMINI_330_PID || pid == GEMINI_336_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_330L_PID || pid == GEMINI_336L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_336LG_PID || pid == GEMINI_335LE_PID ||
         pid == GEMINI_336LE_PID || pid == CUSTOM_ADVANTECH_GEMINI_336_PID ||
         pid == CUSTOM_ADVANTECH_GEMINI_336L_PID || pid == GEMINI_338_PID;
}

bool OBCameraNode::isGemini435LePID(uint32_t pid) {
  const uint16_t GEMINI_435Le_PID = 0x815;
  return pid == GEMINI_435Le_PID;
}

}  // namespace orbbec_camera