/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <cmath>

class SlamGMapping : public rclcpp::Node
{
public:
  SlamGMapping();
  void init();

private:
  std::string base_frame_;
  std::string map_frame_;
  std::string odom_frame_;

  double maxRange_;
  double maxUrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;

  double srr_;
  double srt_;
  double str_;
  double stt_;

  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;

  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double occ_thresh_;

  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;

  int throttle_scans_;
  double map_update_interval_;
  double transform_publish_period_;
  double tf_delay_;
};

SlamGMapping::SlamGMapping()
: Node("slam_gmapping")
{
  init();
}

void SlamGMapping::init()
{
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "odom");

  this->declare_parameter<int>("throttle_scans", 1);
  this->declare_parameter<double>("map_update_interval", 5.0);
  this->declare_parameter<double>("transform_publish_period", 0.05);

  this->declare_parameter<double>("maxRange", 80.0);
  this->declare_parameter<double>("maxUrange", 79.99);
  this->declare_parameter<double>("minimumScore", 0.0);
  this->declare_parameter<double>("sigma", 0.05);
  this->declare_parameter<int>("kernelSize", 1);
  this->declare_parameter<double>("lstep", 0.05);
  this->declare_parameter<double>("astep", 0.05);
  this->declare_parameter<int>("iterations", 5);
  this->declare_parameter<double>("lsigma", 0.075);
  this->declare_parameter<double>("ogain", 3.0);
  this->declare_parameter<int>("lskip", 0);

  this->declare_parameter<double>("srr", 0.1);
  this->declare_parameter<double>("srt", 0.2);
  this->declare_parameter<double>("str", 0.1);
  this->declare_parameter<double>("stt", 0.2);

  this->declare_parameter<double>("linearUpdate", 1.0);
  this->declare_parameter<double>("angularUpdate", 0.5);
  this->declare_parameter<double>("temporalUpdate", -1.0);
  this->declare_parameter<double>("resampleThreshold", 0.5);
  this->declare_parameter<int>("particles", 30);

  this->declare_parameter<double>("xmin", -100.0);
  this->declare_parameter<double>("ymin", -100.0);
  this->declare_parameter<double>("xmax", 100.0);
  this->declare_parameter<double>("ymax", 100.0);
  this->declare_parameter<double>("delta", 0.05);
  this->declare_parameter<double>("occ_thresh", 0.25);

  this->declare_parameter<double>("llsamplerange", 0.01);
  this->declare_parameter<double>("llsamplestep", 0.01);
  this->declare_parameter<double>("lasamplerange", 0.005);
  this->declare_parameter<double>("lasamplestep", 0.005);

  this->declare_parameter<double>("tf_delay", 0.05);

  base_frame_ = this->get_parameter("base_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();

  throttle_scans_ = this->get_parameter("throttle_scans").as_int();
  map_update_interval_ = this->get_parameter("map_update_interval").as_double();
  transform_publish_period_ = this->get_parameter("transform_publish_period").as_double();

  maxRange_ = this->get_parameter("maxRange").as_double();
  maxUrange_ = this->get_parameter("maxUrange").as_double();
  minimum_score_ = this->get_parameter("minimumScore").as_double();
  sigma_ = this->get_parameter("sigma").as_double();
  kernelSize_ = this->get_parameter("kernelSize").as_int();
  lstep_ = this->get_parameter("lstep").as_double();
  astep_ = this->get_parameter("astep").as_double();
  iterations_ = this->get_parameter("iterations").as_int();
  lsigma_ = this->get_parameter("lsigma").as_double();
  ogain_ = this->get_parameter("ogain").as_double();
  lskip_ = this->get_parameter("lskip").as_int();

  srr_ = this->get_parameter("srr").as_double();
  srt_ = this->get_parameter("srt").as_double();
  str_ = this->get_parameter("str").as_double();
  stt_ = this->get_parameter("stt").as_double();

  linearUpdate_ = this->get_parameter("linearUpdate").as_double();
  angularUpdate_ = this->get_parameter("angularUpdate").as_double();
  temporalUpdate_ = this->get_parameter("temporalUpdate").as_double();
  resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
  particles_ = this->get_parameter("particles").as_int();

  xmin_ = this->get_parameter("xmin").as_double();
  ymin_ = this->get_parameter("ymin").as_double();
  xmax_ = this->get_parameter("xmax").as_double();
  ymax_ = this->get_parameter("ymax").as_double();
  delta_ = this->get_parameter("delta").as_double();
  occ_thresh_ = this->get_parameter("occ_thresh").as_double();

  llsamplerange_ = this->get_parameter("llsamplerange").as_double();
  llsamplestep_ = this->get_parameter("llsamplestep").as_double();
  lasamplerange_ = this->get_parameter("lasamplerange").as_double();
  lasamplestep_ = this->get_parameter("lasamplestep").as_double();

  tf_delay_ = this->get_parameter("tf_delay").as_double();

  if (maxUrange_ > maxRange_)
  {
    RCLCPP_WARN(this->get_logger(),
      "maxUrange (%.3f) is greater than maxRange (%.3f). Capping maxUrange to maxRange.",
      maxUrange_, maxRange_);
    maxUrange_ = maxRange_;
    this->set_parameter(rclcpp::Parameter("maxUrange", maxUrange_));
  }

  RCLCPP_INFO(this->get_logger(), "SlamGMapping parameters initialized.");
  RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  particles: %d", particles_);
  RCLCPP_INFO(this->get_logger(), "  delta: %.3f", delta_);
  RCLCPP_INFO(this->get_logger(), "  maxRange: %.3f", maxRange_);
  RCLCPP_INFO(this->get_logger(), "  maxUrange: %.3f", maxUrange_);
  RCLCPP_INFO(this->get_logger(), "  temporalUpdate: %.3f", temporalUpdate_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamGMapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}