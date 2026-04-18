/*
 * Copyright 2013 Open Source Robotics Foundation
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
 *
*/

#include <string>
#include <algorithm>
#include <assert.h>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo_ros/node.hpp>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Distortion.hh>

#include "gazebo_plugins/gazebo_ros_camera_utils.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCameraUtils::GazeboRosCameraUtils()
{
  this->last_update_time_ = common::Time(0);
  this->last_info_update_time_ = common::Time(0);
  this->height_ = 0;
  this->width_ = 0;
  this->skip_ = 0;
  this->format_ = "";
  this->initialized_ = false;
}

void GazeboRosCameraUtils::configCallback(
  gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level)
{
  (void)level;
  if (this->initialized_)
  {
    RCLCPP_INFO(
      this->gazebo_ros_node_->get_logger(),
      "Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f",
      this->camera_name_.c_str(), config.imager_rate);
    this->parentSensor_->SetUpdateRate(config.imager_rate);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);

  if (this->camera_executor_) {
    this->camera_executor_->cancel();
  }

  if (this->callback_queue_thread_.joinable()) {
    this->callback_queue_thread_.join();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  this->Load(_parent, _sdf, _camera_name_suffix);
  this->hack_baseline_ = _hack_baseline;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix)
{
  std::string world_name = _parent->WorldName();
  this->world_ = physics::get_world(world_name);

  this->sdf = _sdf;
  this->world = this->world_;

  std::stringstream ss;
  this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Camera");

  this->gazebo_ros_node_ = gazebo_ros::Node::get(_sdf);

  auto get_sdf_string = [&](const std::string &camel, const std::string &def_val) -> std::string {
    if (_sdf->HasElement(camel)) {
      return _sdf->Get<std::string>(camel);
    }
    return def_val;
  };

  auto get_sdf_double = [&](const std::string &camel, double def_val) -> double {
    if (_sdf->HasElement(camel)) {
      return _sdf->Get<double>(camel);
    }
    return def_val;
  };

  auto get_sdf_bool = [&](const std::string &camel, bool def_val) -> bool {
    if (_sdf->HasElement(camel)) {
      return _sdf->Get<bool>(camel);
    }
    return def_val;
  };

  const std::string sdf_robot_namespace = this->robot_namespace_;
  std::string sdf_camera_name = get_sdf_string("cameraName", _parent->Name());
  if (!_camera_name_suffix.empty()) {
    sdf_camera_name += _camera_name_suffix;
  }

  const std::string sdf_image_topic_name = get_sdf_string("imageTopicName", "image_raw");
  const std::string sdf_camera_info_topic_name = get_sdf_string("cameraInfoTopicName", "camera_info");
  const std::string sdf_trigger_topic_name = get_sdf_string("triggerTopicName", "image_trigger");
  const std::string sdf_frame_name = get_sdf_string("frameName", "camera_link");
  const std::string sdf_tf_prefix = get_sdf_string("tfPrefix", "");

  const double sdf_update_rate = get_sdf_double("updateRate", 0.0);

  const double sdf_cx_prime = get_sdf_double("CxPrime", 0.0);
  const double sdf_cx = get_sdf_double("Cx", 0.0);
  const double sdf_cy = get_sdf_double("Cy", 0.0);
  const double sdf_focal_length = get_sdf_double("focalLength", 0.0);

  const double sdf_distortion_k1 = get_sdf_double("distortionK1", 0.0);
  const double sdf_distortion_k2 = get_sdf_double("distortionK2", 0.0);
  const double sdf_distortion_k3 = get_sdf_double("distortionK3", 0.0);
  const double sdf_distortion_t1 = get_sdf_double("distortionT1", 0.0);
  const double sdf_distortion_t2 = get_sdf_double("distortionT2", 0.0);

  const bool sdf_auto_distortion = get_sdf_bool("autoDistortion", false);
  const bool sdf_border_crop = get_sdf_bool("borderCrop", true);
  const double sdf_hack_baseline = get_sdf_double("hackBaseline", this->hack_baseline_);

  this->robot_namespace_ = this->gazebo_ros_node_->declare_parameter<std::string>("robot_namespace", sdf_robot_namespace);
  this->camera_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("camera_name", sdf_camera_name);
  this->image_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("image_topic_name", sdf_image_topic_name);
  this->camera_info_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("camera_info_topic_name", sdf_camera_info_topic_name);
  this->trigger_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("trigger_topic_name", sdf_trigger_topic_name);
  this->frame_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("frame_name", sdf_frame_name);
  this->tf_prefix_ = this->gazebo_ros_node_->declare_parameter<std::string>("tf_prefix", sdf_tf_prefix);

  this->update_rate_ = this->gazebo_ros_node_->declare_parameter<double>("update_rate", sdf_update_rate);

  this->cx_prime_ = this->gazebo_ros_node_->declare_parameter<double>("cx_prime", sdf_cx_prime);
  this->cx_ = this->gazebo_ros_node_->declare_parameter<double>("cx", sdf_cx);
  this->cy_ = this->gazebo_ros_node_->declare_parameter<double>("cy", sdf_cy);
  this->focal_length_ = this->gazebo_ros_node_->declare_parameter<double>("focal_length", sdf_focal_length);

  this->distortion_k1_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k1", sdf_distortion_k1);
  this->distortion_k2_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k2", sdf_distortion_k2);
  this->distortion_k3_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k3", sdf_distortion_k3);
  this->distortion_t1_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_t1", sdf_distortion_t1);
  this->distortion_t2_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_t2", sdf_distortion_t2);

  this->auto_distortion_ = this->gazebo_ros_node_->declare_parameter<bool>("auto_distortion", sdf_auto_distortion);
  this->border_crop_ = this->gazebo_ros_node_->declare_parameter<bool>("border_crop", sdf_border_crop);
  this->hack_baseline_ = this->gazebo_ros_node_->declare_parameter<double>("hack_baseline", sdf_hack_baseline);

  if (!this->tf_prefix_.empty()) {
    this->frame_name_ = this->tf_prefix_ + "/" + this->frame_name_;
  }

  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "camera_name: %s", this->camera_name_.c_str());
  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "image_topic_name: %s", this->image_topic_name_.c_str());
  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "camera_info_topic_name: %s", this->camera_info_topic_name_.c_str());
  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "trigger_topic_name: %s", this->trigger_topic_name_.c_str());
  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "frame_name: %s", this->frame_name_.c_str());
  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "update_rate: %.6f", this->update_rate_);

  this->image_connect_count_ = std::make_shared<int>(0);
  this->image_connect_count_lock_ = std::make_shared<std::mutex>();
  this->was_active_ = std::make_shared<bool>(false);

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosCameraUtils::LoadThread, this));
}

event::ConnectionPtr GazeboRosCameraUtils::OnLoad(const boost::function<void()>& load_function)
{
  return load_event_.Connect(load_function);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::LoadThread()
{
  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS 2 is not initialized.\n";
    return;
  }

  this->parentSensor_->SetActive(false);

  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(this->gazebo_ros_node_.get(), this->camera_name_));

  RCLCPP_INFO(
    this->gazebo_ros_node_->get_logger(),
    "Camera Plugin (ns = %s) <tf_prefix_>, set to \"%s\"",
    this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  this->image_pub_ = image_transport::create_publisher(
    this->gazebo_ros_node_.get(), this->image_topic_name_);

  this->camera_info_pub_ = this->gazebo_ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    this->camera_info_topic_name_, rclcpp::SensorDataQoS());

  if (this->CanTriggerCamera())
  {
    this->trigger_subscriber_ = this->gazebo_ros_node_->create_subscription<std_msgs::msg::Empty>(
      this->trigger_topic_name_, rclcpp::SensorDataQoS(),
      std::bind(&GazeboRosCameraUtils::TriggerCameraInternal, this, std::placeholders::_1));
  }

  this->camera_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->camera_executor_->add_node(this->gazebo_ros_node_);

  this->Init();
}

void GazeboRosCameraUtils::TriggerCamera()
{
}

bool GazeboRosCameraUtils::CanTriggerCamera()
{
  return false;
}

void GazeboRosCameraUtils::TriggerCameraInternal(
  const std_msgs::msg::Empty::SharedPtr dummy)
{
  (void)dummy;
  TriggerCamera();
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCameraUtils::SetHFOV(const std_msgs::msg::Float64::SharedPtr hfov)
{
#if GAZEBO_MAJOR_VERSION >= 7
  this->camera_->SetHFOV(ignition::math::Angle(hfov->data));
#else
  this->camera_->SetHFOV(gazebo::math::Angle(hfov->data));
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosCameraUtils::SetUpdateRate(
  const std_msgs::msg::Float64::SharedPtr update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::ImageConnect()
{
  std::lock_guard<std::mutex> lock(*this->image_connect_count_lock_);

  if ((*this->image_connect_count_) == 0)
    *this->was_active_ = this->parentSensor_->IsActive();

  (*this->image_connect_count_)++;

  this->parentSensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::ImageDisconnect()
{
  std::lock_guard<std::mutex> lock(*this->image_connect_count_lock_);

  (*this->image_connect_count_)--;

  if ((*this->image_connect_count_) <= 0 && !*this->was_active_)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  if (this->format_ == "L8" || this->format_ == "L_INT8")
  {
    this->type_ = sensor_msgs::image_encodings::MONO8;
    this->skip_ = 1;
  }
  else if (this->format_ == "L16" || this->format_ == "L_INT16")
  {
    this->type_ = sensor_msgs::image_encodings::MONO16;
    this->skip_ = 2;
  }
  else if (this->format_ == "R8G8B8" || this->format_ == "RGB_INT8")
  {
    this->type_ = sensor_msgs::image_encodings::RGB8;
    this->skip_ = 3;
  }
  else if (this->format_ == "B8G8R8" || this->format_ == "BGR_INT8")
  {
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }
  else if (this->format_ == "R16G16B16" || this->format_ == "RGB_INT16")
  {
    this->type_ = sensor_msgs::image_encodings::RGB16;
    this->skip_ = 6;
  }
  else if (this->format_ == "BAYER_RGGB8")
  {
    RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_BGGR8")
  {
    RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GBRG8")
  {
    RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GRBG8")
  {
    RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  }
  else
  {
    RCLCPP_ERROR(this->gazebo_ros_node_->get_logger(), "Unsupported Gazebo ImageFormat");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  if (this->cx_prime_ == 0)
    this->cx_prime_ = (static_cast<double>(this->width_) + 1.0) / 2.0;
  if (this->cx_ == 0)
    this->cx_ = (static_cast<double>(this->width_) + 1.0) / 2.0;
  if (this->cy_ == 0)
    this->cy_ = (static_cast<double>(this->height_) + 1.0) / 2.0;

  double hfov = this->camera_->HFOV().Radian();
  double computed_focal_length =
    (static_cast<double>(this->width_)) /
    (2.0 * tan(hfov / 2.0));

  if (this->focal_length_ == 0)
  {
    this->focal_length_ = computed_focal_length;
  }
  else
  {
    if (!ignition::math::equal(this->focal_length_, computed_focal_length))
    {
      RCLCPP_WARN(
        this->gazebo_ros_node_->get_logger(),
        "The <focal_length>[%f] you have provided for camera_ [%s] is inconsistent with "
        "specified image_width [%d] and HFOV [%f]. Please double check to see that "
        "focal_length = width_ / (2.0 * tan(HFOV/2.0)), the expected focal_length value "
        "is [%f], please update your camera_ model description accordingly.",
        this->focal_length_, this->parentSensor_->Name().c_str(),
        this->width_, hfov, computed_focal_length);
    }
  }

  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = this->frame_name_;
  camera_info_msg.height = this->height_;
  camera_info_msg.width = this->width_;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  if (this->camera_->LensDistortion())
  {
    this->camera_->LensDistortion()->SetCrop(this->border_crop_);
  }

  if (this->auto_distortion_ && this->camera_->LensDistortion())
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->distortion_k1_ = this->camera_->LensDistortion()->K1();
    this->distortion_k2_ = this->camera_->LensDistortion()->K2();
    this->distortion_k3_ = this->camera_->LensDistortion()->K3();
    this->distortion_t1_ = this->camera_->LensDistortion()->P1();
    this->distortion_t2_ = this->camera_->LensDistortion()->P2();
#else
    this->distortion_k1_ = this->camera_->LensDistortion()->GetK1();
    this->distortion_k2_ = this->camera_->LensDistortion()->GetK2();
    this->distortion_k3_ = this->camera_->LensDistortion()->GetK3();
    this->distortion_t1_ = this->camera_->LensDistortion()->GetP1();
    this->distortion_t2_ = this->camera_->LensDistortion()->GetP2();
#endif
  }

  camera_info_msg.d[0] = this->distortion_k1_;
  camera_info_msg.d[1] = this->distortion_k2_;
  camera_info_msg.d[2] = this->distortion_t1_;
  camera_info_msg.d[3] = this->distortion_t2_;
  camera_info_msg.d[4] = this->distortion_k3_;

  camera_info_msg.k[0] = this->focal_length_;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = this->cx_;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = this->focal_length_;
  camera_info_msg.k[5] = this->cy_;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;

  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;

  camera_info_msg.p[0] = this->focal_length_;
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = this->cx_;
  camera_info_msg.p[3] = -this->focal_length_ * this->hack_baseline_;
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = this->focal_length_;
  camera_info_msg.p[6] = this->cy_;
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  this->camera_info_manager_->setCameraInfo(camera_info_msg);

  this->callback_queue_thread_ = std::thread(
    std::bind(&GazeboRosCameraUtils::CameraQueueThread, this));

  load_event_();
  this->initialized_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src,
  common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src)
{
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
    return;

  if (this->image_pub_.getNumSubscribers() > 0)
  {
    std::lock_guard<std::mutex> lock(this->lock_);

    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = static_cast<int32_t>(this->sensor_update_time_.sec);
    this->image_msg_.header.stamp.nanosec = static_cast<uint32_t>(this->sensor_update_time_.nsec);

    sensor_msgs::fillImage(
      this->image_msg_, this->type_, this->height_, this->width_,
      this->skip_ * this->width_, reinterpret_cast<const void*>(_src));

    this->image_pub_.publish(this->image_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PublishCameraInfo(common::Time &last_update_time)
{
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
    return;

  this->sensor_update_time_ = last_update_time;
  this->PublishCameraInfo();
}

void GazeboRosCameraUtils::PublishCameraInfo()
{
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
    return;

  if (this->camera_info_pub_->get_subscription_count() > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
    if (this->sensor_update_time_ - this->last_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->camera_info_pub_);
      this->last_info_update_time_ = this->sensor_update_time_;
    }
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher)
{
  sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();

  camera_info_msg.header.stamp.sec = static_cast<int32_t>(this->sensor_update_time_.sec);
  camera_info_msg.header.stamp.nanosec = static_cast<uint32_t>(this->sensor_update_time_.nsec);

  camera_info_publisher->publish(camera_info_msg);
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  static const auto timeout = std::chrono::milliseconds(1);

  while (rclcpp::ok())
  {
    if (this->camera_executor_) {
      this->camera_executor_->spin_some();
    }
    std::this_thread::sleep_for(timeout);
  }
}

}  // namespace gazebo