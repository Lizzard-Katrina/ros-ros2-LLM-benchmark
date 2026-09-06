// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef DEPTH_IMAGE_PROC__CONVERSIONS_HPP_
#define DEPTH_IMAGE_PROC__CONVERSIONS_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depth_image_proc
{

// Build a lookup table of 3D unit vectors for each pixel, accounting for
// radial distortion. When multiplied by depth, each vector gives the 3D point.
inline cv::Mat initMatrix(
  cv::Mat cameraMatrix, cv::Mat distCoeffs,
  int width, int height, bool radial)
{
  int count = width * height;
  cv::Mat mapDist(count, 1, CV_32FC2);
  cv::Mat map(count, 1, CV_32FC2);

  for (int j = 0, i = 0; j < height; j++) {
    for (int k = 0; k < width; k++, i++) {
      mapDist.at<cv::Vec2f>(i, 0)[0] = static_cast<float>(k);
      mapDist.at<cv::Vec2f>(i, 0)[1] = static_cast<float>(j);
    }
  }

  // Undistort pixel coordinates
  cv::undistortPoints(mapDist, map, cameraMatrix, distCoeffs);

  cv::Mat transform(height, width, CV_32FC3);
  for (int j = 0, i = 0; j < height; j++) {
    for (int k = 0; k < width; k++, i++) {
      float ux = map.at<cv::Vec2f>(i, 0)[0];
      float uy = map.at<cv::Vec2f>(i, 0)[1];
      float mag = std::sqrt(ux * ux + uy * uy + 1.0f);
      if (radial) {
        transform.at<cv::Vec3f>(j, k)[0] = ux / mag;
        transform.at<cv::Vec3f>(j, k)[1] = uy / mag;
        transform.at<cv::Vec3f>(j, k)[2] = 1.0f / mag;
      } else {
        transform.at<cv::Vec3f>(j, k)[0] = ux;
        transform.at<cv::Vec3f>(j, k)[1] = uy;
        transform.at<cv::Vec3f>(j, k)[2] = 1.0f;
      }
    }
  }
  return transform;
}

// Convert depth image to point cloud using radial transform
template<typename T>
void convertDepthRadial(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  sensor_msgs::msg::PointCloud2 & cloud_msg,
  const cv::Mat & transform)
{
  // Use correct depth conversion factor
  float bad_point = std::numeric_limits<float>::quiet_NaN();
  float unit_scaling;
  if (typeid(T) == typeid(uint16_t)) {
    unit_scaling = 0.001f;  // mm to m
  } else {
    unit_scaling = 1.0f;
  }

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);

  for (int v = 0; v < static_cast<int>(depth_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(depth_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
      T raw_depth = depth_row[u];
      if (raw_depth == 0) {
        *iter_x = *iter_y = *iter_z = bad_point;
        continue;
      }
      float depth = static_cast<float>(raw_depth) * unit_scaling;
      cv::Vec3f t = transform.at<cv::Vec3f>(v, u);
      *iter_x = t[0] * depth;
      *iter_y = t[1] * depth;
      *iter_z = t[2] * depth;
    }
  }
}

// Fill in RGB data from an image message into a PointCloud2
inline void convertRgb(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  sensor_msgs::msg::PointCloud2 & cloud_msg,
  int red_offset, int green_offset, int blue_offset, int color_step)
{
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

  const uint8_t * rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  for (int v = 0; v < static_cast<int>(rgb_msg->height); ++v, rgb += rgb_skip) {
    for (int u = 0; u < static_cast<int>(rgb_msg->width);
      ++u, rgb += color_step, ++iter_r, ++iter_g, ++iter_b)
    {
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}

}  // namespace depth_image_proc

#endif  // DEPTH_IMAGE_PROC__CONVERSIONS_HPP_