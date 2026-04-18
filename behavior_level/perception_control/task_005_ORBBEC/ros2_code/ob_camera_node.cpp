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

// ... (unchanged code above)

void OBCameraNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
  if (!is_running_) {
    ROS_WARN_ONCE("Frame callback called before initialization");
    return;
  }
  if (!isInitialized()) {
    ROS_WARN_ONCE("Frame callback called before initialization");
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  ROS_INFO_STREAM_ONCE("Received first frame set");
  try {
    if (enable_point_cloud_ || enable_colored_point_cloud_ || enable_d2c_viewer_) {
      publishPointCloud(frame_set);
    }

    if (enable_stream_[DEPTH]) {
      auto depth = frame_set->depthFrame();
      if (depth) {
        auto depth_frame = std::shared_ptr<ob::Frame>(depth);
        if (enable_depth_filter_) {
          auto filtered = processDepthFrameFilter(depth_frame);
          if (filtered) {
            depth_frame = filtered;
          }
        }
        onNewFrameCallback(depth_frame, DEPTH);
      }
    }

    if (enable_stream_[INFRA0]) {
      auto ir = frame_set->irFrame();
      if (ir) {
        auto ir_frame = std::shared_ptr<ob::Frame>(ir);
        auto decoded = decodeIRMJPGFrame(ir_frame);
        if (decoded) {
          ir_frame = decoded;
        }
        onNewFrameCallback(ir_frame, INFRA0);
      }
    }

    if (enable_stream_[INFRA1]) {
      auto ir_left = frame_set->getFrame(OB_FRAME_IR_LEFT);
      if (ir_left) {
        auto left_ir_frame = std::shared_ptr<ob::Frame>(ir_left);
        auto decoded = decodeIRMJPGFrame(left_ir_frame);
        if (decoded) {
          left_ir_frame = decoded;
        }
        auto filtered = processLeftIrFrameFilter(left_ir_frame);
        if (filtered) {
          left_ir_frame = filtered;
        }
        onNewFrameCallback(left_ir_frame, INFRA1);
      }
    }

    if (enable_stream_[INFRA2]) {
      auto ir_right = frame_set->getFrame(OB_FRAME_IR_RIGHT);
      if (ir_right) {
        auto right_ir_frame = std::shared_ptr<ob::Frame>(ir_right);
        auto decoded = decodeIRMJPGFrame(right_ir_frame);
        if (decoded) {
          right_ir_frame = decoded;
        }
        auto filtered = processRightIrFrameFilter(right_ir_frame);
        if (filtered) {
          right_ir_frame = filtered;
        }
        onNewFrameCallback(right_ir_frame, INFRA2);
      }
    }

    if (enable_stream_[COLOR]) {
      auto color = frame_set->colorFrame();
      if (color) {
        std::lock_guard<std::mutex> lk(colorFrameMtx_);
        colorFrameQueue_.push(frame_set);
        colorFrameCV_.notify_one();
      }
    }

    if (enable_stream_[COLOR_LEFT]) {
      auto color_left = frame_set->getFrame(OB_FRAME_COLOR_LEFT);
      if (color_left) {
        std::lock_guard<std::mutex> lk(leftColorFrameMtx_);
        leftColorFrameQueue_.push(frame_set);
        leftColorFrameCV_.notify_one();
      }
    }

    if (enable_stream_[COLOR_RIGHT]) {
      auto color_right = frame_set->getFrame(OB_FRAME_COLOR_RIGHT);
      if (color_right) {
        std::lock_guard<std::mutex> lk(rightColorFrameMtx_);
        rightColorFrameQueue_.push(frame_set);
        rightColorFrameCV_.notify_one();
      }
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: unknown error");
  }
}

void OBCameraNode::onNewColorFrameCallback() {
  while (enable_stream_[COLOR] && rclcpp::ok() && is_running_.load()) {
    std::shared_ptr<ob::FrameSet> fs = nullptr;
    {
      std::unique_lock<std::mutex> lock(colorFrameMtx_);
      colorFrameCV_.wait(lock, [this]() {
        return !colorFrameQueue_.empty() || !is_running_.load() || !rclcpp::ok();
      });
      if (!is_running_.load() || !rclcpp::ok()) {
        break;
      }
      fs = colorFrameQueue_.front();
      colorFrameQueue_.pop();
    }

    if (!fs) {
      continue;
    }

    auto color = fs->colorFrame();
    if (!color) {
      continue;
    }

    auto frame = std::shared_ptr<ob::Frame>(color);
    auto filtered = processColorFrameFilter(frame);
    if (filtered) {
      frame = filtered;
    }

    rgb_is_decoded_ = decodeColorFrameToBuffer(frame, rgb_buffer_);
    onNewFrameCallback(frame, COLOR);
  }

  ROS_INFO_STREAM("Color frame thread exit!");
}

void OBCameraNode::onNewLeftColorFrameCallback() {
  while (enable_stream_[COLOR_LEFT] && rclcpp::ok() && is_running_.load()) {
    std::shared_ptr<ob::FrameSet> fs = nullptr;
    {
      std::unique_lock<std::mutex> lock(leftColorFrameMtx_);
      leftColorFrameCV_.wait(lock, [this]() {
        return !leftColorFrameQueue_.empty() || !is_running_.load() || !rclcpp::ok();
      });
      if (!is_running_.load() || !rclcpp::ok()) {
        break;
      }
      fs = leftColorFrameQueue_.front();
      leftColorFrameQueue_.pop();
    }

    if (!fs) {
      continue;
    }

    auto color_left = fs->getFrame(OB_FRAME_COLOR_LEFT);
    if (!color_left) {
      continue;
    }

    auto frame = std::shared_ptr<ob::Frame>(color_left);
    auto filtered = processColorFrameFilter(frame);
    if (filtered) {
      frame = filtered;
    }

    rgb_left_is_decoded_ = decodeColorFrameToBuffer(frame, rgb_buffer_left_);
    onNewFrameCallback(frame, COLOR_LEFT);
  }

  ROS_INFO_STREAM("Left Color frame thread exit!");
}

void OBCameraNode::onNewRightColorFrameCallback() {
  while (enable_stream_[COLOR_RIGHT] && rclcpp::ok() && is_running_.load()) {
    std::shared_ptr<ob::FrameSet> fs = nullptr;
    {
      std::unique_lock<std::mutex> lock(rightColorFrameMtx_);
      rightColorFrameCV_.wait(lock, [this]() {
        return !rightColorFrameQueue_.empty() || !is_running_.load() || !rclcpp::ok();
      });
      if (!is_running_.load() || !rclcpp::ok()) {
        break;
      }
      fs = rightColorFrameQueue_.front();
      rightColorFrameQueue_.pop();
    }

    if (!fs) {
      continue;
    }

    auto color_right = fs->getFrame(OB_FRAME_COLOR_RIGHT);
    if (!color_right) {
      continue;
    }

    auto frame = std::shared_ptr<ob::Frame>(color_right);
    auto filtered = processColorFrameFilter(frame);
    if (filtered) {
      frame = filtered;
    }

    rgb_right_is_decoded_ = decodeColorFrameToBuffer(frame, rgb_buffer_right_);
    onNewFrameCallback(frame, COLOR_RIGHT);
  }

  ROS_INFO_STREAM("Right Color frame thread exit!");
}

void OBCameraNode::onNewFrameCallback(std::shared_ptr<ob::Frame> frame,
                                      const stream_index_pair& stream_index) {
  if (frame == nullptr) {
    return;
  }
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (camera_info_publishers_.count(stream_index) > 0 &&
      camera_info_publishers_[stream_index].getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (metadata_publishers_.count(stream_index) > 0 &&
      metadata_publishers_[stream_index].getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return;
  }

  auto video_frame = frame->as<ob::VideoFrame>();
  if (!video_frame) {
    return;
  }

  uint32_t width = video_frame->width();
  uint32_t height = video_frame->height();
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
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else if (ir_camera_info_manager_ && ir_camera_info_manager_->isCalibrated() &&
             (stream_index == INFRA0 || stream_index == DEPTH)) {
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    auto camera_info = ir_camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else {
    OBCameraIntrinsic intrinsic;
    OBCameraDistortion distortion;
    CHECK_NOTNULL(device_info_.get());
    if (isGemini335PID(device_info_->pid())) {
      auto stream_profile = frame->getStreamProfile();
      CHECK_NOTNULL(stream_profile.get());
      auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(video_stream_profile);
      intrinsic = video_stream_profile->getIntrinsic();
      distortion = video_stream_profile->getDistortion();
    } else {
      auto camera_params = pipeline_->getCameraParam();
      intrinsic = stream_index == COLOR ? camera_params.rgbIntrinsic : camera_params.depthIntrinsic;
      distortion =
          stream_index == COLOR ? camera_params.rgbDistortion : camera_params.depthDistortion;
      if (device_info_->pid() == DABAI_MAX_PID) {
        intrinsic = camera_params.rgbIntrinsic;
        distortion = camera_params.rgbDistortion;
      }
    }
    auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
    CHECK(camera_info_publishers_.count(stream_index) > 0);
    auto camera_info_publisher = camera_info_publishers_[stream_index];

    if (stream_index == COLOR_RIGHT && stream_profile_.count(COLOR_LEFT) > 0 &&
        stream_profile_[COLOR_LEFT]) {
      auto left_video_profile = stream_profile_[COLOR_LEFT]->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(left_video_profile.get());
      auto stream_profile = frame->getStreamProfile();
      CHECK_NOTNULL(stream_profile.get());
      auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(video_stream_profile.get());
      auto ex = video_stream_profile->getExtrinsicTo(left_video_profile);
      double fx = camera_info.K.at(0);
      double fy = camera_info.K.at(4);
      camera_info.P.at(3) = fx * ex.trans[0] / 1000.0 + 0.0;
      camera_info.P.at(7) = fy * ex.trans[1] / 1000.0 + 0.0;
    }

    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  }

  CHECK(image_publishers_.count(stream_index));
  if (!image_publishers_[stream_index].getNumSubscribers()) {
    return;
  }
  auto& image = images_[stream_index];

  std_msgs::Header header;
  header.stamp = timestamp;
  header.frame_id = frame_id;

  std::string encoding;
  cv::Mat cv_image;

  auto format = frame->format();
  if (stream_index == COLOR || stream_index == COLOR_LEFT || stream_index == COLOR_RIGHT) {
    const uint8_t* data_ptr = nullptr;
    if (stream_index == COLOR && rgb_is_decoded_ && rgb_buffer_) {
      data_ptr = rgb_buffer_;
      cv_image = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(data_ptr));
      encoding = sensor_msgs::image_encodings::BGRA8;
    } else if (stream_index == COLOR_LEFT && rgb_left_is_decoded_ && rgb_buffer_left_) {
      data_ptr = rgb_buffer_left_;
      cv_image = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(data_ptr));
      encoding = sensor_msgs::image_encodings::BGRA8;
    } else if (stream_index == COLOR_RIGHT && rgb_right_is_decoded_ && rgb_buffer_right_) {
      data_ptr = rgb_buffer_right_;
      cv_image = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(data_ptr));
      encoding = sensor_msgs::image_encodings::BGRA8;
    } else {
      auto decoded = softwareDecodeColorFrame(frame);
      if (!decoded) {
        return;
      }
      auto decoded_video = decoded->as<ob::VideoFrame>();
      if (!decoded_video) {
        return;
      }
      format = decoded->format();
      width = decoded_video->width();
      height = decoded_video->height();
      switch (format) {
        case OB_FORMAT_RGB:
        case OB_FORMAT_RGB888:
          encoding = sensor_msgs::image_encodings::RGB8;
          cv_image = cv::Mat(height, width, CV_8UC3,
                             const_cast<void*>(reinterpret_cast<const void*>(decoded_video->data())));
          break;
        case OB_FORMAT_BGR:
        case OB_FORMAT_BGR888:
          encoding = sensor_msgs::image_encodings::BGR8;
          cv_image = cv::Mat(height, width, CV_8UC3,
                             const_cast<void*>(reinterpret_cast<const void*>(decoded_video->data())));
          break;
        case OB_FORMAT_RGBA:
          encoding = sensor_msgs::image_encodings::RGBA8;
          cv_image = cv::Mat(height, width, CV_8UC4,
                             const_cast<void*>(reinterpret_cast<const void*>(decoded_video->data())));
          break;
        case OB_FORMAT_BGRA:
          encoding = sensor_msgs::image_encodings::BGRA8;
          cv_image = cv::Mat(height, width, CV_8UC4,
                             const_cast<void*>(reinterpret_cast<const void*>(decoded_video->data())));
          break;
        default:
          return;
      }
    }
  } else {
    switch (format) {
      case OB_FORMAT_Y16:
      case OB_FORMAT_Z16:
        encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        cv_image = cv::Mat(height, width, CV_16UC1,
                           const_cast<void*>(reinterpret_cast<const void*>(video_frame->data())));
        break;
      case OB_FORMAT_Y8:
        encoding = sensor_msgs::image_encodings::MONO8;
        cv_image = cv::Mat(height, width, CV_8UC1,
                           const_cast<void*>(reinterpret_cast<const void*>(video_frame->data())));
        break;
      default:
        return;
    }
  }

  if (cv_image.empty()) {
    return;
  }

  cv::Mat output = cv_image;
  if (image_flip_[stream_index] || image_mirror_[stream_index]) {
    int flip_code = 0;
    if (image_flip_[stream_index] && image_mirror_[stream_index]) {
      flip_code = -1;
    } else if (image_mirror_[stream_index]) {
      flip_code = 1;
    } else {
      flip_code = 0;
    }
    cv::flip(cv_image, output, flip_code);
  }

  image = output.clone();
  auto image_msg = cv_bridge::CvImage(header, encoding, output).toImageMsg();
  image_publishers_[stream_index].publish(image_msg);

  saveImageToFile(stream_index, image, image_msg);
}

// ... (unchanged code below)

}  // namespace orbbec_camera
