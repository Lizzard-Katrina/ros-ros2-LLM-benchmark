class AmclNode : public rclcpp::Node
{
  public:
    AmclNode();
    ~AmclNode();

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     * @param in_bag_fn input bagfile
     * @param trigger_global_localization whether to trigger global localization
     * before starting to process the bagfile
     */
    void runFromBag(const std::string &in_bag_fn, bool trigger_global_localization = false);

    int process();
    void savePoseToServer();

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    bool sent_first_transform_;

    tf2::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Callbacks
    bool globalLocalizationCallback(std_srvs::srv::Empty::Request::SharedPtr req,
                                    std_srvs::srv::Empty::Response::SharedPtr res);
    bool nomotionUpdateCallback(std_srvs::srv::Empty::Request::SharedPtr req,
                                    std_srvs::srv::Empty::Response::SharedPtr res);
    bool setMapCallback(nav_msgs::srv::SetMap::Request::SharedPtr req,
                        nav_msgs::srv::SetMap::Response::SharedPtr res);

    void laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::msg::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();

    //parameter for which odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    geometry_msgs::msg::PoseStamped latest_odom_pose_;

    //parameter for which base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    rclcpp::Duration gui_publish_period;
    rclcpp::Time save_pose_last_time;
    rclcpp::Duration save_pose_period;

    geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan>* laser_scan_sub_;
    tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>* laser_scan_filter_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    rclcpp::Duration cloud_pub_interval;
    rclcpp::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    rclcpp::Duration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(geometry_msgs::msg::PoseStamped& pose,
                     double& x, double& y, double& yaw,
                     const rclcpp::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    rclcpp::Duration transform_tolerance_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_; //to let amcl update samples without requiring motion
    rclcpp::Service<nav_msgs::srv::SetMap>::SharedPtr set_map_srv_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_old_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    diagnostic_updater::Updater diagnosic_updater_;
    void standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);
    double std_warn_level_x_;
    double std_warn_level_y_;
    double std_warn_level_yaw_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    amcl::AMCLConfig default_config_;
    rclcpp::TimerBase::SharedPtr check_laser_timer_;

    int max_particles_, min_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;
    bool force_update_after_initialpose_;
    bool force_update_after_set_map_;
    bool selective_resampling_;

    rcl_interfaces::msg::SetParametersResult on_params_set(const std::vector<rclcpp::Parameter> &params);

    rclcpp::Time last_laser_received_ts_;
    rclcpp::Duration laser_check_interval_;
    void checkLaserReceived();
};

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int,int> > AmclNode::free_space_indices;
#endif

AmclNode::AmclNode() :
        Node("amcl"),
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(nullptr),
        pf_(nullptr),
        resample_count_(0),
        odom_(nullptr),
        laser_(nullptr),
        initial_pose_hyp_(nullptr),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Declare parameters and get initial values
  min_particles_ = this->declare_parameter<int>("min_particles", 100);
  max_particles_ = this->declare_parameter<int>("max_particles", 5000);
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  d_thresh_ = this->declare_parameter<double>("update_min_d", 0.2);

  // Register parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AmclNode::on_params_set, this, std::placeholders::_1));

  // Setup dynamic reconfigure server equivalent
  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(this);
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = 
    std::bind(&AmclNode::reconfigureCB, this, std::placeholders::_1, std::placeholders::_2);
  dsrv_->setCallback(cb);

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = rclcpp::Duration::from_seconds(15.0);
  check_laser_timer_ = this->create_wall_timer(
    laser_check_interval_,
    std::bind(&AmclNode::checkLaserReceived, this));

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::standardDeviationDiagnostics);
}

rcl_interfaces::msg::SetParametersResult AmclNode::on_params_set(const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  int new_min_particles = min_particles_;
  int new_max_particles = max_particles_;

  for (const auto &param : params)
  {
    if (param.get_name() == "min_particles")
    {
      new_min_particles = param.as_int();
    }
    else if (param.get_name() == "max_particles")
    {
      new_max_particles = param.as_int();
    }
  }

  if (new_min_particles > new_max_particles)
  {
    result.successful = false;
    result.reason = "min_particles cannot be greater than max_particles";
    return result;
  }

  // If validation passed, update internal members
  for (const auto &param : params)
  {
    if (param.get_name() == "min_particles")
    {
      min_particles_ = param.as_int();
    }
    else if (param.get_name() == "max_particles")
    {
      max_particles_ = param.as_int();
    }
    else if (param.get_name() == "odom_frame_id")
    {
      odom_frame_id_ = param.as_string();
    }
    else if (param.get_name() == "update_min_d")
    {
      d_thresh_ = param.as_double();
    }
  }

  return result;
}

void AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
  // This function is deprecated in ROS2, replaced by on_params_set callback
  // but to keep compatibility, we update parameters here as well
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  if (config.min_particles > config.max_particles)
  {
    RCLCPP_WARN(this->get_logger(), "min_particles cannot be greater than max_particles");
    return;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  odom_frame_id_ = config.odom_frame_id;
  d_thresh_ = config.update_min_d;

  // Update other parameters as needed...
}