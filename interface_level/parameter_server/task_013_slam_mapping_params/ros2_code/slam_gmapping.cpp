void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();

  // 1. TF Frames: base_frame, map_frame, odom_frame.
  private_nh_.declare_parameter<std::string>("base_frame", "base_link");
  private_nh_.declare_parameter<std::string>("map_frame", "map");
  private_nh_.declare_parameter<std::string>("odom_frame", "odom");

  private_nh_.get_parameter("base_frame", base_frame_);
  private_nh_.get_parameter("map_frame", map_frame_);
  private_nh_.get_parameter("odom_frame", odom_frame_);

  // 2. Scanner Limits: maxRange, maxUrange, minimumScore.
  private_nh_.declare_parameter<double>("maxRange", 0.0);
  private_nh_.declare_parameter<double>("maxUrange", 0.0);
  private_nh_.declare_parameter<double>("minimumScore", 0.0);

  private_nh_.get_parameter("maxRange", maxRange_);
  private_nh_.get_parameter("maxUrange", maxUrange_);
  private_nh_.get_parameter("minimumScore", minimum_score_);

  // Validate laser range parameters
  if (maxRange_ <= 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("slam_gmapping"), "maxRange parameter is not set or invalid, defaulting to 0.0");
  }
  if (maxUrange_ <= 0.0)
  {
    maxUrange_ = maxRange_;
    RCLCPP_INFO(rclcpp::get_logger("slam_gmapping"), "maxUrange not set or invalid, defaulting to maxRange: %f", maxUrange_);
  }
  if (maxUrange_ > maxRange_)
  {
    RCLCPP_WARN(rclcpp::get_logger("slam_gmapping"), "maxUrange (%f) is greater than maxRange (%f), adjusting maxUrange to maxRange", maxUrange_, maxRange_);
    maxUrange_ = maxRange_;
  }

  // 3. Motion Model (Gaussian Noise): srr, srt, str, stt.
  private_nh_.declare_parameter<double>("srr", 0.1);
  private_nh_.declare_parameter<double>("srt", 0.2);
  private_nh_.declare_parameter<double>("str", 0.1);
  private_nh_.declare_parameter<double>("stt", 0.2);

  private_nh_.get_parameter("srr", srr_);
  private_nh_.get_parameter("srt", srt_);
  private_nh_.get_parameter("str", str_);
  private_nh_.get_parameter("stt", stt_);

  // 4. Grid Resolution: xmin, ymin, xmax, ymax, delta.
  private_nh_.declare_parameter<double>("xmin", -10.0);
  private_nh_.declare_parameter<double>("ymin", -10.0);
  private_nh_.declare_parameter<double>("xmax", 10.0);
  private_nh_.declare_parameter<double>("ymax", 10.0);
  private_nh_.declare_parameter<double>("delta", 0.05);

  private_nh_.get_parameter("xmin", xmin_);
  private_nh_.get_parameter("ymin", ymin_);
  private_nh_.get_parameter("xmax", xmax_);
  private_nh_.get_parameter("ymax", ymax_);
  private_nh_.get_parameter("delta", delta_);

  // 5. Update Strategy: linearUpdate, angularUpdate, temporalUpdate, particles.
  private_nh_.declare_parameter<double>("linearUpdate", 1.0);
  private_nh_.declare_parameter<double>("angularUpdate", 0.5);
  private_nh_.declare_parameter<double>("temporalUpdate", 1.0);
  private_nh_.declare_parameter<int>("particles", 30);

  private_nh_.get_parameter("linearUpdate", linearUpdate_);
  private_nh_.get_parameter("angularUpdate", angularUpdate_);
  private_nh_.get_parameter("temporalUpdate", temporalUpdate_);
  private_nh_.get_parameter("particles", particles_);

  // Additional parameters used in GMapping
  private_nh_.declare_parameter<double>("sigma", 0.05);
  private_nh_.declare_parameter<int>("kernelSize", 1);
  private_nh_.declare_parameter<double>("lstep", 0.05);
  private_nh_.declare_parameter<double>("astep", 0.05);
  private_nh_.declare_parameter<int>("iterations", 5);
  private_nh_.declare_parameter<double>("lsigma", 0.075);
  private_nh_.declare_parameter<double>("ogain", 3.0);
  private_nh_.declare_parameter<int>("lskip", 0);
  private_nh_.declare_parameter<double>("resampleThreshold", 0.5);
  private_nh_.declare_parameter<double>("llsamplerange", 0.01);
  private_nh_.declare_parameter<double>("lasamplerange", 0.005);
  private_nh_.declare_parameter<double>("llsamplestep", 0.01);
  private_nh_.declare_parameter<double>("lasamplestep", 0.005);
  private_nh_.declare_parameter<double>("map_update_interval", 5.0);
  private_nh_.declare_parameter<double>("occ_thresh", 0.25);

  private_nh_.get_parameter("sigma", sigma_);
  private_nh_.get_parameter("kernelSize", kernelSize_);
  private_nh_.get_parameter("lstep", lstep_);
  private_nh_.get_parameter("astep", astep_);
  private_nh_.get_parameter("iterations", iterations_);
  private_nh_.get_parameter("lsigma", lsigma_);
  private_nh_.get_parameter("ogain", ogain_);
  private_nh_.get_parameter("lskip", lskip_);
  private_nh_.get_parameter("resampleThreshold", resampleThreshold_);
  private_nh_.get_parameter("llsamplerange", llsamplerange_);
  private_nh_.get_parameter("lasamplerange", lasamplerange_);
  private_nh_.get_parameter("llsamplestep", llsamplestep_);
  private_nh_.get_parameter("lasamplestep", lasamplestep_);
  private_nh_.get_parameter("map_update_interval", map_update_interval_);
  private_nh_.get_parameter("occ_thresh", occ_thresh_);

  // Initialize tf buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Initialize map publisher and service will be done in startLiveSlam or startReplay

  // Initialize other members
  got_first_scan_ = false;
  got_map_ = false;
  throttle_scans_ = 1;
  private_nh_.declare_parameter<int>("throttle_scans", throttle_scans_);
  private_nh_.get_parameter("throttle_scans", throttle_scans_);

  transform_publish_period_ = 0.05;
  private_nh_.declare_parameter<double>("transform_publish_period", transform_publish_period_);
  private_nh_.get_parameter("transform_publish_period", transform_publish_period_);

  tf_delay_ = transform_publish_period_;

  // Assign tf_ to tf_buffer_ for compatibility
  tf_ = *tf_buffer_;
}