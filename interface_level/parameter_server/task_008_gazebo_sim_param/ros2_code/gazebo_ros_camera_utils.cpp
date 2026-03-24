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
    RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f",
             this->camera_name_.c_str(), config.imager_rate);
    this->parentSensor_->SetUpdateRate(config.imager_rate);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->gazebo_ros_node_.reset();
  this->camera_queue_.clear();
  this->camera_queue_.disable();
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

  // TODO
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

  // Initialize gazebo_ros_node_
  this->gazebo_ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Declare and get parameters with SDF defaults
  // Camera name
  std::string camera_name_default = "";
  if (_sdf->HasElement("cameraName"))
    camera_name_default = _sdf->Get<std::string>("cameraName");
  this->camera_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("camera_name", camera_name_default);

  // Image topic name
  std::string image_topic_default = "image_raw";
  if (_sdf->HasElement("imageTopicName"))
    image_topic_default = _sdf->Get<std::string>("imageTopicName");
  this->image_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("image_topic_name", image_topic_default);

  // Camera info topic name
  std::string camera_info_topic_default = "camera_info";
  if (_sdf->HasElement("cameraInfoTopicName"))
    camera_info_topic_default = _sdf->Get<std::string>("cameraInfoTopicName");
  this->camera_info_topic_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("camera_info_topic_name", camera_info_topic_default);

  // Frame name
  std::string frame_name_default = "";
  if (_sdf->HasElement("frameName"))
    frame_name_default = _sdf->Get<std::string>("frameName");
  this->frame_name_ = this->gazebo_ros_node_->declare_parameter<std::string>("frame_name", frame_name_default);

  // Update rate
  double update_rate_default = 0.0;
  if (_sdf->HasElement("updateRate"))
    update_rate_default = _sdf->Get<double>("updateRate");
  this->update_rate_ = this->gazebo_ros_node_->declare_parameter<double>("update_rate", update_rate_default);

  // Format
  std::string format_default = "";
  if (_sdf->HasElement("format"))
    format_default = _sdf->Get<std::string>("format");
  this->format_ = this->gazebo_ros_node_->declare_parameter<std::string>("format", format_default);

  // Optical parameters
  double cx_prime_default = 0.0;
  if (_sdf->HasElement("cxPrime"))
    cx_prime_default = _sdf->Get<double>("cxPrime");
  this->cx_prime_ = this->gazebo_ros_node_->declare_parameter<double>("cx_prime", cx_prime_default);

  double cx_default = 0.0;
  if (_sdf->HasElement("cx"))
    cx_default = _sdf->Get<double>("cx");
  this->cx_ = this->gazebo_ros_node_->declare_parameter<double>("cx", cx_default);

  double cy_default = 0.0;
  if (_sdf->HasElement("cy"))
    cy_default = _sdf->Get<double>("cy");
  this->cy_ = this->gazebo_ros_node_->declare_parameter<double>("cy", cy_default);

  double focal_length_default = 0.0;
  if (_sdf->HasElement("focalLength"))
    focal_length_default = _sdf->Get<double>("focalLength");
  this->focal_length_ = this->gazebo_ros_node_->declare_parameter<double>("focal_length", focal_length_default);

  // Distortion coefficients
  double distortion_k1_default = 0.0;
  if (_sdf->HasElement("distortionK1"))
    distortion_k1_default = _sdf->Get<double>("distortionK1");
  this->distortion_k1_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k1", distortion_k1_default);

  double distortion_k2_default = 0.0;
  if (_sdf->HasElement("distortionK2"))
    distortion_k2_default = _sdf->Get<double>("distortionK2");
  this->distortion_k2_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k2", distortion_k2_default);

  double distortion_k3_default = 0.0;
  if (_sdf->HasElement("distortionK3"))
    distortion_k3_default = _sdf->Get<double>("distortionK3");
  this->distortion_k3_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_k3", distortion_k3_default);

  double distortion_t1_default = 0.0;
  if (_sdf->HasElement("distortionT1"))
    distortion_t1_default = _sdf->Get<double>("distortionT1");
  this->distortion_t1_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_t1", distortion_t1_default);

  double distortion_t2_default = 0.0;
  if (_sdf->HasElement("distortionT2"))
    distortion_t2_default = _sdf->Get<double>("distortionT2");
  this->distortion_t2_ = this->gazebo_ros_node_->declare_parameter<double>("distortion_t2", distortion_t2_default);

  // Border crop
  bool border_crop_default = true;
  if (_sdf->HasElement("borderCrop"))
    border_crop_default = _sdf->Get<bool>("borderCrop");
  this->border_crop_ = this->gazebo_ros_node_->declare_parameter<bool>("border_crop", border_crop_default);

  // Auto distortion flag
  bool auto_distortion_default = true;
  if (_sdf->HasElement("autoDistortion"))
    auto_distortion_default = _sdf->Get<bool>("autoDistortion");
  this->auto_distortion_ = this->gazebo_ros_node_->declare_parameter<bool>("auto_distortion", auto_distortion_default);

  RCLCPP_DEBUG(this->gazebo_ros_node_->get_logger(), "Camera Plugin (ns = %s) loaded with camera_name: %s, image_topic_name: %s, frame_name: %s",
    this->robot_namespace_.c_str(), this->camera_name_.c_str(), this->image_topic_name_.c_str(), this->frame_name_.c_str());

  this->deferred_load_thread_ = std::thread(
    std::bind(&GazeboRosCameraUtils::LoadThread, this));
}

event::ConnectionPtr GazeboRosCameraUtils::OnLoad(const std::function<void()>& load_function)
{
  return load_event_.Connect(load_function);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::LoadThread()
{
  // Exit if no ROS
  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // Sensor generation off by default.  Must do this before advertising the
  // associated ROS topics.
  this->parentSensor_->SetActive(false);

  // initialize camera_info_manager
  this->camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
          this->gazebo_ros_node_, this->camera_name_);

  this->itnode_ = std::make_shared<image_transport::ImageTransport>(this->gazebo_ros_node_);

  // resolve tf prefix
  this->tf_prefix_ = this->gazebo_ros_node_->declare_parameter<std::string>("tf_prefix", "");
  if (!this->tf_prefix_.empty())
  {
    this->frame_name_ = this->tf_prefix_ + "/" + this->frame_name_;
  }

  RCLCPP_INFO(this->gazebo_ros_node_->get_logger(), "Camera Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  if (!this->camera_name_.empty())
  {
    // dynamic reconfigure is not directly supported in ROS2, skipping or implement using parameters callback if needed
    // For now, no dynamic reconfigure server
  }
  else
  {
    RCLCPP_WARN(this->gazebo_ros_node_->get_logger(), "dynamic reconfigure is not enabled for this image topic [%s]"
             " because <cameraName> is not specified",
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
  // same timestamps.  This incurs additional computational cost when
  // there are subscribers to camera_info, but better mimics behavior
  // of image_pipeline.
  rclcpp::PublisherOptions camera_info_pub_options;
  camera_info_pub_options.callback_group = this->gazebo_ros_node_->get_callback_group();
  this->camera_info_pub_ = this->gazebo_ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    this->camera_info_topic_name_, rclcpp::QoS(2).transient_local());

  /*
  // disabling fov and rate setting for each camera
  this->cameraHFOVSubscriber_ = this->gazebo_ros_node_->create_subscription<std_msgs::msg::Float64>(
    "set_hfov", 1,
    std::bind(&GazeboRosCameraUtils::SetHFOV, this, std::placeholders::_1));

  this->cameraUpdateRateSubscriber_ = this->gazebo_ros_node_->create_subscription<std_msgs::msg::Float64>(
    "set_update_rate", 1,
    std::bind(&GazeboRosCameraUtils::SetUpdateRate, this, std::placeholders::_1));
  */

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
    const std_msgs::msg::Empty::SharedPtr /*dummy*/)
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
               " the expected focal_length value is [%f],"
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
  // distortion black border. The crop can be useful, but also skews
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
    // TODO: remove version guard once gazebo7 is not supported
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
    this->image_msg_.header.stamp = this->gazebo_ros_node_->now();

    // copy from src to image_msg_
    sensor_msgs::fillImage(this->image_msg_, this->type_, this->height_, this->width_,
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

  camera_info_msg.header.stamp = this->gazebo_ros_node_->now();

  camera_info_publisher->publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  rclcpp::Rate rate(1000); // 1000 Hz

  while (rclcpp::ok())
  {
    /// take care of callback queue
    this->camera_queue_.callAvailable(rclcpp::Duration::from_seconds(0.001));
    rate.sleep();
  }
}
}