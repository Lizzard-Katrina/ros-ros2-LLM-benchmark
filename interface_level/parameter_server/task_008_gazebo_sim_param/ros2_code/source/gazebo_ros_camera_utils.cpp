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
#include <thread>
#include <mutex>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Distortion.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gazebo_plugins/gazebo_ros_camera_utils.h"

namespace gazebo
{

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

GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->camera_queue_thread_.join();
}

void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  this->Load(_parent, _sdf, _camera_name_suffix);

  this->hack_baseline_ = _hack_baseline;
}

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

  this->gazebo_ros_node_ = gazebo_ros::Node::Get(_sdf);

  auto logger = this->gazebo_ros_node_->get_logger();

  std::string image_topic_default = "image_raw";
  if (this->sdf->HasElement("imageTopicName"))
    image_topic_default = this->sdf->Get<std::string>("imageTopicName");
  this->gazebo_ros_node_->declare_parameter<std::string>("image_topic_name", image_topic_default);
  this->image_topic_name_ = this->gazebo_ros_node_->get_parameter("image_topic_name").as_string();

  std::string trigger_topic_default = "image_trigger";
  if (this->sdf->HasElement("triggerTopicName"))
    trigger_topic_default = this->sdf->Get<std::string>("triggerTopicName");
  this->gazebo_ros_node_->declare_parameter<std::string>("trigger_topic_name", trigger_topic_default);
  this->trigger_topic_name_ = this->gazebo_ros_node_->get_parameter("trigger_topic_name").as_string();

  std::string camera_info_topic_default = "camera_info";
  if (this->sdf->HasElement("cameraInfoTopicName"))
    camera_info_topic_default = this->sdf->Get<std::string>("cameraInfoTopicName");
  this->gazebo_ros_node_->declare_parameter<std::string>("camera_info_topic_name", camera_info_topic_default);
  this->camera_info_topic_name_ = this->gazebo_ros_node_->get_parameter("camera_info_topic_name").as_string();

  std::string camera_name_default = "";
  if (!this->sdf->HasElement("cameraName"))
    RCLCPP_DEBUG(logger, "Camera plugin missing <cameraName>, default to empty");
  else
    camera_name_default = this->sdf->Get<std::string>("cameraName");
  this->gazebo_ros_node_->declare_parameter<std::string>("camera_name", camera_name_default);
  this->camera_name_ = this->gazebo_ros_node_->get_parameter("camera_name").as_string();

  this->camera_name_ += _camera_name_suffix;

  std::string frame_name_default = "/world";
  if (!this->sdf->HasElement("frameName"))
    RCLCPP_DEBUG(logger, "Camera plugin missing <frameName>, defaults to /world");
  else
    frame_name_default = this->sdf->Get<std::string>("frameName");
  this->gazebo_ros_node_->declare_parameter<std::string>("frame_name", frame_name_default);
  this->frame_name_ = this->gazebo_ros_node_->get_parameter("frame_name").as_string();

  double update_rate_default = 0.0;
  if (!this->sdf->HasElement("updateRate"))
  {
    RCLCPP_DEBUG(logger, "Camera plugin missing <updateRate>, defaults to unlimited (0).");
  }
  else
    update_rate_default = this->sdf->Get<double>("updateRate");
  this->gazebo_ros_node_->declare_parameter<double>("update_rate", update_rate_default);
  this->update_rate_ = this->gazebo_ros_node_->get_parameter("update_rate").as_double();

  double cx_prime_default = 0.0;
  if (!this->sdf->HasElement("CxPrime"))
  {
    RCLCPP_DEBUG(logger, "Camera plugin missing <CxPrime>, defaults to 0");
  }
  else
    cx_prime_default = this->sdf->Get<double>("CxPrime");
  this->gazebo_ros_node_->declare_parameter<double>("cx_prime", cx_prime_default);
  this->cx_prime_ = this->gazebo_ros_node_->get_parameter("cx_prime").as_double();

  double cx_default = 0.0;
  if (!this->sdf->HasElement("Cx"))
  {
    RCLCPP_DEBUG(logger, "Camera plugin missing <Cx>, defaults to 0");
  }
  else
    cx_default = this->sdf->Get<double>("Cx");
  this->gazebo_ros_node_->declare_parameter<double>("cx", cx_default);
  this->cx_ = this->gazebo_ros_node_->get_parameter("cx").as_double();

  double cy_default = 0.0;
  if (!this->sdf->HasElement("Cy"))
  {
    RCLCPP_DEBUG(logger, "Camera plugin missing <Cy>, defaults to 0");
  }
  else
    cy_default = this->sdf->Get<double>("Cy");
  this->gazebo_ros_node_->declare_parameter<double>("cy", cy_default);
  this->cy_ = this->gazebo_ros_node_->get_parameter("cy").as_double();

  double focal_length_default = 0.0;
  if (!this->sdf->HasElement("focalLength"))
  {
    RCLCPP_DEBUG(logger, "Camera plugin missing <focalLength>, defaults to 0");
  }
  else
    focal_length_default = this->sdf->Get<double>("focalLength");
  this->gazebo_ros_node_->declare_parameter<double>("focal_length", focal_length_default);
  this->focal_length_ = this->gazebo_ros_node_->get_parameter("focal_length").as_double();

  this->hack_baseline_ = 0.0;
  if (this->sdf->HasElement("hackBaseline"))
    this->hack_baseline_ = this->sdf->Get<double>("hackBaseline");
  this->gazebo_ros_node_->declare_parameter<double>("hack_baseline", this->hack_baseline_);
  this->hack_baseline_ = this->gazebo_ros_node_->get_parameter("hack_baseline").as_double();

  double distortion_k1_default = 0.0;
  if (this->sdf->HasElement("distortionK1"))
    distortion_k1_default = this->sdf->Get<double>("distortionK1");
  this->gazebo_ros_node_->declare_parameter<double>("distortion_k1", distortion_k1_default);
  this->distortion_k1_ = this->gazebo_ros_node_->get_parameter("distortion_k1").as_double();

  double distortion_k2_default = 0.0;
  if (this->sdf->HasElement("distortionK2"))
    distortion_k2_default = this->sdf->Get<double>("distortionK2");
  this->gazebo_ros_node_->declare_parameter<double>("distortion_k2", distortion_k2_default);
  this->distortion_k2_ = this->gazebo_ros_node_->get_parameter("distortion_k2").as_double();

  double distortion_k3_default = 0.0;
  if (this->sdf->HasElement("distortionK3"))
    distortion_k3_default = this->sdf->Get<double>("distortionK3");
  this->gazebo_ros_node_->declare_parameter<double>("distortion_k3", distortion_k3_default);
  this->distortion_k3_ = this->gazebo_ros_node_->get_parameter("distortion_k3").as_double();

  double distortion_t1_default = 0.0;
  if (this->sdf->HasElement("distortionT1"))
    distortion_t1_default = this->sdf->Get<double>("distortionT1");
  this->gazebo_ros_node_->declare_parameter<double>("distortion_t1", distortion_t1_default);
  this->distortion_t1_ = this->gazebo_ros_node_->get_parameter("distortion_t1").as_double();

  double distortion_t2_default = 0.0;
  if (this->sdf->HasElement("distortionT2"))
    distortion_t2_default = this->sdf->Get<double>("distortionT2");
  this->gazebo_ros_node_->declare_parameter<double>("distortion_t2", distortion_t2_default);
  this->distortion_t2_ = this->gazebo_ros_node_->get_parameter("distortion_t2").as_double();

  bool auto_distortion_default = false;
  if (this->sdf->HasElement("autoDistortion"))
    auto_distortion_default = this->sdf->Get<bool>("autoDistortion");
  this->gazebo_ros_node_->declare_parameter<bool>("auto_distortion", auto_distortion_default);
  this->auto_distortion_ = this->gazebo_ros_node_->get_parameter("auto_distortion").as_bool();

  bool border_crop_default = true;
  if (this->sdf->HasElement("borderCrop"))
    border_crop_default = this->sdf->Get<bool>("borderCrop");
  this->gazebo_ros_node_->declare_parameter<bool>("border_crop", border_crop_default);
  this->border_crop_ = this->gazebo_ros_node_->get_parameter("border_crop").as_bool();

  this->image_connect_count_ = std::shared_ptr<int>(new int(0));
  this->image_connect_count_lock_ = std::shared_ptr<std::mutex>(new std::mutex);
  this->was_active_ = std::shared_ptr<bool>(new bool(false));

  this->deferred_load_thread_ = std::thread(
    std::bind(&GazeboRosCameraUtils::LoadThread, this));
}

event::ConnectionPtr GazeboRosCameraUtils::OnLoad(const std::function<void()>& load_function)
{
  return load_event_.Connect(load_function);
}

void GazeboRosCameraUtils::LoadThread()
{
  this->parentSensor_->SetActive(false);

  this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          this->gazebo_ros_node_.get(), this->camera_name_));

  this->itnode_ = std::make_shared<image_transport::ImageTransport>(
    std::shared_ptr<rclcpp::Node>(this->gazebo_ros_node_));

  RCLCPP_INFO(this->gazebo_ros_node_->get_logger(),
    "Camera Plugin (ns = %s)", this->robot_namespace_.c_str());

  this->image_pub_ = this->itnode_->advertise(
    this->image_topic_name_, 2);

  this->camera_info_pub_ = this->gazebo_ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    this->camera_info_topic_name_, 2);

  if (this->CanTriggerCamera())
  {
    this->trigger_subscriber_ = this->gazebo_ros_node_->create_subscription<std_msgs::msg::Empty>(
      this->trigger_topic_name_, 1,
      std::bind(&GazeboRosCameraUtils::TriggerCameraInternal, this, std::placeholders::_1));
  }

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
  TriggerCamera();
}

void GazeboRosCameraUtils::SetHFOV(const std_msgs::msg::Float64::SharedPtr hfov)
{
  this->camera_->SetHFOV(ignition::math::Angle(hfov->data));
}

void GazeboRosCameraUtils::SetUpdateRate(
  const std_msgs::msg::Float64::SharedPtr update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

void GazeboRosCameraUtils::ImageConnect()
{
  std::lock_guard<std::mutex> lock(*this->image_connect_count_lock_);

  if ((*this->image_connect_count_) == 0)
    *this->was_active_ = this->parentSensor_->IsActive();

  (*this->image_connect_count_)++;

  this->parentSensor_->SetActive(true);
}

void GazeboRosCameraUtils::ImageDisconnect()
{
  std::lock_guard<std::mutex> lock(*this->image_connect_count_lock_);

  (*this->image_connect_count_)--;

  if ((*this->image_connect_count_) <= 0 && !*this->was_active_)
    this->parentSensor_->SetActive(false);
}

void GazeboRosCameraUtils::Init()
{
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
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
    this->cx_prime_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cx_ == 0)
    this->cx_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cy_ == 0)
    this->cy_ = (static_cast<double>(this->height_) + 1.0) /2.0;

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
      RCLCPP_WARN(this->gazebo_ros_node_->get_logger(),
               "The <focal_length>[%f] you have provided for camera_ [%s]"
               " is inconsistent with specified image_width [%d] and"
               " HFOV [%f].   Please double check to see that"
               " focal_length = width_ / (2.0 * tan(HFOV/2.0)),"
               " the expected focal_length value is [%f],"
               " please update your camera_ model description accordingly.",
                this->focal_length_, this->parentSensor_->Name().c_str(),
                this->width_, hfov,
                computed_focal_length);
    }
  }

  sensor_msgs::msg::CameraInfo camera_info_msg;

  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  if(this->camera_->LensDistortion())
  {
    this->camera_->LensDistortion()->SetCrop(this->border_crop_);
  }

  if(this->auto_distortion_)
  {
    this->distortion_k1_ = this->camera_->LensDistortion()->K1();
    this->distortion_k2_ = this->camera_->LensDistortion()->K2();
    this->distortion_k3_ = this->camera_->LensDistortion()->K3();
    this->distortion_t1_ = this->camera_->LensDistortion()->P1();
    this->distortion_t2_ = this->camera_->LensDistortion()->P2();
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

  this->camera_queue_thread_ = std::thread(
    std::bind(&GazeboRosCameraUtils::CameraQueueThread, this));

  load_event_();
  this->initialized_ = true;
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src,
  common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  if ((*this->image_connect_count_) > 0)
  {
    std::lock_guard<std::mutex> lock(this->lock_);

    this->image_msg_.header.frame_id = this->frame_name_;

    rclcpp::Time ros_time = this->gazebo_ros_node_->now();
    this->image_msg_.header.stamp = ros_time;

    sensor_msgs::fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(_src));

    this->image_pub_.publish(this->image_msg_);
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(common::Time &last_update_time)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->sensor_update_time_ = last_update_time;
  this->PublishCameraInfo();
}

void GazeboRosCameraUtils::PublishCameraInfo()
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
  if (this->sensor_update_time_ - this->last_info_update_time_ >= this->update_period_)
  {
    this->PublishCameraInfo(this->camera_info_pub_);
    this->last_info_update_time_ = this->sensor_update_time_;
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher)
{
  sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();

  rclcpp::Time ros_time = this->gazebo_ros_node_->now();
  camera_info_msg.header.stamp = ros_time;

  camera_info_publisher->publish(camera_info_msg);
}

void GazeboRosCameraUtils::CameraQueueThread()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->gazebo_ros_node_);

  while (rclcpp::ok())
  {
    executor.spin_some(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}