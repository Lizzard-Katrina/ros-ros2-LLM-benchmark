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

FILE_PATH: gazebo_ros_camera_utils.cpp
----------------------------
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
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <camera_info_manager/camera_info_manager.h>

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
  if (this->initialized_)
  {
    ROS_INFO_NAMED("camera_utils", "Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f",
             this->camera_name_.c_str(), config.imager_rate);
    this->parentSensor_->SetUpdateRate(config.imager_rate);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->rosnode_->shutdown();
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  // default Load:
  // provide _camera_name_suffix to prevent LoadThread() creating the ros::NodeHandle with
  //an incomplete this->camera_name_ namespace. There was a race condition when the _camera_name_suffix
  //was appended in this function.
  this->Load(_parent, _sdf, _camera_name_suffix);

  // overwrite hack baseline if specified at load
  // example usage in gazebo_ros_multicamera
  this->hack_baseline_ = _hack_baseline;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix)
{
  // Get the world name.
  std::string world_name = _parent->WorldName();

  // Get the world_
  this->world_ = physics::get_world(world_name);

  // save pointers
  this->sdf = _sdf;

  // maintain for one more release for backwards compatibility with
  // pr2_gazebo_plugins
  this->world = this->world_;

  std::stringstream ss;
  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Camera");

  //TODO
  // [Objective]: 
// Migrate the legacy SDF parsing and ROS 1 initialization to the ROS 2 gazebo_ros::Node system.
//
// [Requirements]:
// 1. Initialize 'gazebo_ros_node_' using 'gazebo_ros::Node::get(_sdf)'.
// 2. Declare and retrieve all camera configuration parameters (topics, names, frames, and update rates).
// 3. Migrate the optical and distortion parameters (Cx, Cy, Focal Length, and K/T coefficients).
// 4. Implement a logic where SDF values act as defaults, but can be overridden by ROS 2 parameters.
// 5. Ensure correct mapping from SDF CamelCase tags to ROS 2 snake_case parameter names.
//
// [Constraints]:
// - Use 'this->gazebo_ros_node_->declare_parameter<T>(...)' for all declarations.
// - Replace all 'ROS_DEBUG_NAMED' with 'RCLCPP_DEBUG' using the node's logger.
// - Ensure 'boost::shared_ptr' for synchronization primitives are converted to 'std::shared_ptr'.
  //END OF TODO
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
  // Exit if no ROS
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // Sensor generation off by default.  Must do this before advertising the
  // associated ROS topics.
  this->parentSensor_->SetActive(false);

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_ + "/" + this->camera_name_);

  // initialize camera_info_manager
  this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          *this->rosnode_, this->camera_name_));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  // resolve tf prefix
  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  ROS_INFO_NAMED("camera_utils", "Camera Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());


  if (!this->camera_name_.empty())
  {
    dyn_srv_ =
      new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
      (*this->rosnode_);
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
      ::CallbackType f =
      boost::bind(&GazeboRosCameraUtils::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    dyn_srv_->setCallback(f);
  }
  else
  {
    ROS_WARN_NAMED("camera_utils", "dynamic reconfigure is not enabled for this image topic [%s]"
             " becuase <cameraName> is not specified",
             this->image_topic_name_.c_str());
  }

  this->image_pub_ = this->itnode_->advertise(
    this->image_topic_name_, 2,
    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
    ros::VoidPtr(), true);

  // camera info publish rate will be synchronized to image sensor
  // publish rates.
  // If someone connects to camera_info, sensor will be activated
  // and camera_info will be published alongside image_raw with the
  // same timestamps.  This incurrs additional computational cost when
  // there are subscribers to camera_info, but better mimics behavior
  // of image_pipeline.
  ros::AdvertiseOptions cio =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->camera_info_topic_name_, 2,
    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
    ros::VoidPtr(), &this->camera_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(cio);

  /* disabling fov and rate setting for each camera
  ros::SubscribeOptions zoom_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_hfov", 1,
        boost::bind(&GazeboRosCameraUtils::SetHFOV, this, boost::placeholders::_1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraHFOVSubscriber_ = this->rosnode_->subscribe(zoom_so);

  ros::SubscribeOptions rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_update_rate", 1,
        boost::bind(&GazeboRosCameraUtils::SetUpdateRate, this, boost::placeholders::_1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraUpdateRateSubscriber_ = this->rosnode_->subscribe(rate_so);
  */

  if (this->CanTriggerCamera())
  {
    ros::SubscribeOptions trigger_so =
      ros::SubscribeOptions::create<std_msgs::Empty>(
          this->trigger_topic_name_, 1,
          boost::bind(&GazeboRosCameraUtils::TriggerCameraInternal, this, boost::placeholders::_1),
          ros::VoidPtr(), &this->camera_queue_);
    this->trigger_subscriber_ = this->rosnode_->subscribe(trigger_so);
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
    const std_msgs::Empty::ConstPtr &dummy)
{
  TriggerCamera();
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCameraUtils::SetHFOV(const std_msgs::Float64::ConstPtr& hfov)
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
  const std_msgs::Float64::ConstPtr& update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::ImageConnect()
{
  boost::mutex::scoped_lock lock(*this->image_connect_count_lock_);

  // upon first connection, remember if camera was active.
  if ((*this->image_connect_count_) == 0)
    *this->was_active_ = this->parentSensor_->IsActive();

  (*this->image_connect_count_)++;

  this->parentSensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::ImageDisconnect()
{
  boost::mutex::scoped_lock lock(*this->image_connect_count_lock_);

  (*this->image_connect_count_)--;

  // if there are no more subscribers, but camera was active to begin with,
  // leave it active.  Use case:  this could be a multicamera, where
  // each camera shares the same parentSensor_.
  if ((*this->image_connect_count_) <= 0 && !*this->was_active_)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  // set buffer size
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
  else if (this->format_ == "R16G16B16" ||  this->format_ == "RGB_INT16")
  {
    this->type_ = sensor_msgs::image_encodings::RGB16;
    this->skip_ = 6;
  }
  else if (this->format_ == "BAYER_RGGB8")
  {
    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_BGGR8")
  {
    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GBRG8")
  {
    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GRBG8")
  {
    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  }
  else
  {
    ROS_ERROR_NAMED("camera_utils", "Unsupported Gazebo ImageFormat\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  /// Compute camera_ parameters if set to 0
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
    // check against float precision
    if (!ignition::math::equal(this->focal_length_, computed_focal_length))
    {
      ROS_WARN_NAMED("camera_utils", "The <focal_length>[%f] you have provided for camera_ [%s]"
               " is inconsistent with specified image_width [%d] and"
               " HFOV [%f].   Please double check to see that"
               " focal_length = width_ / (2.0 * tan(HFOV/2.0)),"
               " the explected focal_lengtth value is [%f],"
               " please update your camera_ model description accordingly.",
                this->focal_length_, this->parentSensor_->Name().c_str(),
                this->width_, hfov,
                computed_focal_length);
    }
  }

  // fill CameraInfo
  sensor_msgs::CameraInfo camera_info_msg;

  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.D.resize(5);
#endif
  // Allow the user to disable automatic cropping (used to remove barrel
  // distortion black border. The crop can be useful, but also skewes
  // the lens distortion, making the supplied k and t values incorrect.
  if(this->camera_->LensDistortion())
  {
    this->camera_->LensDistortion()->SetCrop(this->border_crop_);
  }

  // Get distortion parameters from gazebo sensor if auto_distortion is true
  if(this->auto_distortion_)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->distortion_k1_ = this->camera_->LensDistortion()->K1();
    this->distortion_k2_ = this->camera_->LensDistortion()->K2();
    this->distortion_k3_ = this->camera_->LensDistortion()->K3();
    this->distortion_t1_ = this->camera_->LensDistortion()->P1();
    this->distortion_t2_ = this->camera_->LensDistortion()->P2();
#else
    // TODO: remove version gaurd once gazebo7 is not supported
    this->distortion_k1_ = this->camera_->LensDistortion()->GetK1();
    this->distortion_k2_ = this->camera_->LensDistortion()->GetK2();
    this->distortion_k3_ = this->camera_->LensDistortion()->GetK3();
    this->distortion_t1_ = this->camera_->LensDistortion()->GetP1();
    this->distortion_t2_ = this->camera_->LensDistortion()->GetP2();
#endif
  }

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.D[0] = this->distortion_k1_;
  camera_info_msg.D[1] = this->distortion_k2_;
  camera_info_msg.D[2] = this->distortion_t1_;
  camera_info_msg.D[3] = this->distortion_t2_;
  camera_info_msg.D[4] = this->distortion_k3_;
  // original camera_ matrix
  camera_info_msg.K[0] = this->focal_length_;
  camera_info_msg.K[1] = 0.0;
  camera_info_msg.K[2] = this->cx_;
  camera_info_msg.K[3] = 0.0;
  camera_info_msg.K[4] = this->focal_length_;
  camera_info_msg.K[5] = this->cy_;
  camera_info_msg.K[6] = 0.0;
  camera_info_msg.K[7] = 0.0;
  camera_info_msg.K[8] = 1.0;
  // rectification
  camera_info_msg.R[0] = 1.0;
  camera_info_msg.R[1] = 0.0;
  camera_info_msg.R[2] = 0.0;
  camera_info_msg.R[3] = 0.0;
  camera_info_msg.R[4] = 1.0;
  camera_info_msg.R[5] = 0.0;
  camera_info_msg.R[6] = 0.0;
  camera_info_msg.R[7] = 0.0;
  camera_info_msg.R[8] = 1.0;
  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.P[0] = this->focal_length_;
  camera_info_msg.P[1] = 0.0;
  camera_info_msg.P[2] = this->cx_;
  camera_info_msg.P[3] = -this->focal_length_ * this->hack_baseline_;
  camera_info_msg.P[4] = 0.0;
  camera_info_msg.P[5] = this->focal_length_;
  camera_info_msg.P[6] = this->cy_;
  camera_info_msg.P[7] = 0.0;
  camera_info_msg.P[8] = 0.0;
  camera_info_msg.P[9] = 0.0;
  camera_info_msg.P[10] = 1.0;
  camera_info_msg.P[11] = 0.0;

  this->camera_info_manager_->setCameraInfo(camera_info_msg);

  // start custom queue for camera_
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosCameraUtils::CameraQueueThread, this));

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
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  /// don't bother if there are no subscribers
  if ((*this->image_connect_count_) > 0)
  {
    boost::mutex::scoped_lock lock(this->lock_);

    // copy data into image
    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
    this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

    // copy from src to image_msg_
    fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(_src));

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
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

  if (this->camera_info_pub_.getNumSubscribers() > 0)
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
  ros::Publisher camera_info_publisher)
{
  sensor_msgs::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;

  camera_info_publisher.publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode_->ok())
  {
    /// take care of callback queue
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}

----------------------------


# LLM Response

[FILENAME: gazebo_ros_camera_utils.cpp]
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
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
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
#include <gazebo_ros/utils.hpp>

#include "gazebo_plugins/gazebo_ros_camera_utils.hpp"

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
  if (this->initialized_)
  {
    RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f",
             this->camera_name_.c_str(), config.imager_rate);
    this->parentSensor_->SetUpdateRate(config.imager_rate);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->camera_queue_.reset();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  // default Load:
  // provide _camera_name_suffix to prevent LoadThread() creating the ros::NodeHandle with
  //an incomplete this->camera_name_ namespace. There was a race condition when the _camera_name_suffix
  //was appended in this function.
  this->Load(_parent, _sdf, _camera_name_suffix);

  // overwrite hack baseline if specified at load
  // example usage in gazebo_ros_multicamera
  this->hack_baseline_ = _hack_baseline;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix)
{
  // Get the world name.
  std::string world_name = _parent->WorldName();

  // Get the world_
  this->world_ = physics::get_world(world_name);

  // save pointers
  this->sdf = _sdf;

  // maintain for one more release for backwards compatibility with
  // pr2_gazebo_plugins
  this->world = this->world_;

  std::stringstream ss;
  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Camera");

  this->gazebo_ros_node_ = gazebo_ros::Node::get(_sdf);

  this->image_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "image_topic_name", _sdf->Get<std::string>("imageTopicName", "image_raw"));
  this->camera_info_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "camera_info_topic_name", _sdf->Get<std::string>("cameraInfoTopicName", "camera_info"));
  this->camera_name_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "camera_name", _sdf->Get<std::string>("cameraName", ""));
  this->frame_name_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "frame_name", _sdf->Get<std::string>("frameName", "camera"));
  this->update_rate_ = this->gazebo_ros_node_->declare_parameter<double>(
    "update_rate", _sdf->Get<double>("updateRate", 0.0));
  this->format_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "format", _sdf->Get<std::string>("imageFormat", "R8G8B8"));
  this->border_crop_ = this->gazebo_ros_node_->declare_parameter<bool>(
    "border_crop", _sdf->Get<bool>("borderCrop", true));
  this->auto_distortion_ = this->gazebo_ros_node_->declare_parameter<bool>(
    "auto_distortion", _sdf->Get<bool>("autoDistortion", false));

  this->cx_prime_ = this->gazebo_ros_node_->declare_parameter<double>(
    "cx_prime", _sdf->Get<double>("CxPrime", 0.0));
  this->cx_ = this->gazebo_ros_node_->declare_parameter<double>(
    "cx", _sdf->Get<double>("Cx", 0.0));
  this->cy_ = this->gazebo_ros_node_->declare_parameter<double>(
    "cy", _sdf->Get<double>("Cy", 0.0));
  this->focal_length_ = this->gazebo_ros_node_->declare_parameter<double>(
    "focal_length", _sdf->Get<double>("focalLength", 0.0));

  this->distortion_k1_ = this->gazebo_ros_node_->declare_parameter<double>(
    "distortion_k1", _sdf->Get<double>("distortionK1", 0.0));
  this->distortion_k2_ = this->gazebo_ros_node_->declare_parameter<double>(
    "distortion_k2", _sdf->Get<double>("distortionK2", 0.0));
  this->distortion_k3_ = this->gazebo_ros_node_->declare_parameter<double>(
    "distortion_k3", _sdf->Get<double>("distortionK3", 0.0));
  this->distortion_t1_ = this->gazebo_ros_node_->declare_parameter<double>(
    "distortion_t1", _sdf->Get<double>("distortionT1", 0.0));
  this->distortion_t2_ = this->gazebo_ros_node_->declare_parameter<double>(
    "distortion_t2", _sdf->Get<double>("distortionT2", 0.0));

  this->trigger_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>(
    "trigger_topic_name", _sdf->Get<std::string>("triggerTopicName", "trigger"));

  if (!_camera_name_suffix.empty()) {
    this->camera_name_ += _camera_name_suffix;
  }

  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(),
    "Camera name: %s, Image topic: %s, Info topic: %s, Frame: %s",
    this->camera_name_.c_str(), this->image_topic_name_.c_str(),
    this->camera_info_topic_name_.c_str(), this->frame_name_.c_str());

  this->image_connect_count_lock_ = std::make_shared<std::mutex>();
  this->image_connect_count_ = std::make_shared<int>(0);
  this->was_active_ = std::make_shared<bool>(false);

  this->deferred_load_thread_ = std::thread(
    std::bind(&GazeboRosCameraUtils::LoadThread, this));
}

event::ConnectionPtr GazeboRosCameraUtils::OnLoad(const boost::function<void()>& load_function)
{
  return load_event_.Connect(load_function);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::LoadThread()
{
  // Sensor generation off by default.  Must do this before advertising the
  // associated ROS topics.
  this->parentSensor_->SetActive(false);

  // initialize camera_info_manager
  this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          this->gazebo_ros_node_, this->camera_name_));

  this->itnode_ = std::make_shared<image_transport::ImageTransport>(this->gazebo_ros_node_);

  // resolve tf prefix
  this->tf_prefix_ = gazebo_ros::get_namespace(this->gazebo_ros_node_);
  this->frame_name_ = tf2::get_frame_id(this->frame_name_, this->tf_prefix_);

  RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "Camera Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());


  if (!this->camera_name_.empty())
  {
    // Dynamic reconfigure is typically replaced by ROS 2 parameters in ROS 2.
    // We can optionally add a parameter callback here if needed, but the parameters
    // are already declared and can be changed via ROS 2 parameter services.
  }
  else
  {
    RCLCPP_WARN(this->gazebo_ros_node_->get_logger(), "dynamic reconfigure is not enabled for this image topic [%s]"
             " becuase <cameraName> is not specified",
             this->image_topic_name_.c_str());
  }

  this->image_pub_ = this->itnode_->advertise(
    this->image_topic_name_, 2,
    std::bind(&GazeboRosCameraUtils::ImageConnect, this),
    std::bind(&GazeboRosCameraUtils::ImageDisconnect, this));

  // camera info publish rate will be synchronized to image sensor
  // publish rates.
  // If someone connects to camera_info, sensor will be activated
  // and camera_info will be published alongside image_raw with the
  // same timestamps.  This incurrs additional computational cost when
  // there are subscribers to camera_info, but better mimics behavior
  // of image_pipeline.
  this->camera_info_pub_ = this->gazebo_ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    this->camera_info_topic_name_, rclcpp::SensorDataQoS());

  if (this->CanTriggerCamera())
  {
    this->trigger_subscriber_ = this->gazebo_ros_node_->create_subscription<std_msgs::msg::Empty>(
        this->trigger_topic_name_, rclcpp::SensorDataQoS(),
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

  // upon first connection, remember if camera was active.
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

  // if there are no more subscribers, but camera was active to begin with,
  // leave it active.  Use case:  this could be a multicamera, where
  // each camera shares the same parentSensor_.
  if ((*this->image_connect_count_) <= 0 && !*this->was_active_)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  // set buffer size
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
  else if (this->format_ == "R16G16B16" ||  this->format_ == "RGB_INT16")
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
    RCLCPP_ERROR(this->gazebo_ros_node_->get_logger(), "Unsupported Gazebo ImageFormat\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  /// Compute camera_ parameters if set to 0
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
    // check against float precision
    if (!ignition::math::equal(this->focal_length_, computed_focal_length))
    {
      RCLCPP_WARN(this->gazebo_ros_node_->get_logger(), "The <focal_length>[%f] you have provided for camera_ [%s]"
               " is inconsistent with specified image_width [%d] and"
               " HFOV [%f].   Please double check to see that"
               " focal_length = width_ / (2.0 * tan(HFOV/2.0)),"
               " the explected focal_lengtth value is [%f],"
               " please update your camera_ model description accordingly.",
                this->focal_length_, this->parentSensor_->Name().c_str(),
                this->width_, hfov,
                computed_focal_length);
    }
  }

  // fill CameraInfo
  sensor_msgs::msg::CameraInfo camera_info_msg;

  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  // distortion
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);
  // Allow the user to disable automatic cropping (used to remove barrel
  // distortion black border. The crop can be useful, but also skewes
  // the lens distortion, making the supplied k and t values incorrect.
  if(this->camera_->LensDistortion())
  {
    this->camera_->LensDistortion()->SetCrop(this->border_crop_);
  }

  // Get distortion parameters from gazebo sensor if auto_distortion is true
  if(this->auto_distortion_)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->distortion_k1_ = this->camera_->LensDistortion()->K1();
    this->distortion_k2_ = this->camera_->LensDistortion()->K2();
    this->distortion_k3_ = this->camera_->LensDistortion()->K3();
    this->distortion_t1_ = this->camera_->LensDistortion()->P1();
    this->distortion_t2_ = this->camera_->LensDistortion()->P2();
#else
    // TODO: remove version gaurd once gazebo7 is not supported
    this->distortion_k1_ = this->camera_->LensDistortion()->GetK1();
    this->distortion_k2_ = this->camera_->LensDistortion()->GetK2();
    this->distortion_k3_ = this->camera_->LensDistortion()->GetK3();
    this->distortion_t1_ = this->camera_->LensDistortion()->GetP1();
    this->distortion_t2_ = this->camera_->LensDistortion()->GetP2();
#endif
  }

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.d[0] = this->distortion_k1_;
  camera_info_msg.d[1] = this->distortion_k2_;
  camera_info_msg.d[2] = this->distortion_t1_;
  camera_info_msg.d[3] = this->distortion_t2_;
  camera_info_msg.d[4] = this->distortion_k3_;
  // original camera_ matrix
  camera_info_msg.k[0] = this->focal_length_;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = this->cx_;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = this->focal_length_;
  camera_info_msg.k[5] = this->cy_;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;
  // rectification
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;
  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
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

  // start custom queue for camera_
  this->camera_queue_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->camera_queue_->add_node(this->gazebo_ros_node_);
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
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  /// don't bother if there are no subscribers
  if ((*this->image_connect_count_) > 0)
  {
    std::lock_guard<std::mutex> lock(this->lock_);

    // copy data into image
    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
    this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

    // copy from src to image_msg_
    fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(_src));

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
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

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;

  camera_info_publisher->publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  static const double timeout = 0.001;

  while (rclcpp::ok())
  {
    /// take care of callback queue
    this->camera_queue_->spin_some(std::chrono::duration<double>(timeout));
  }
}
}