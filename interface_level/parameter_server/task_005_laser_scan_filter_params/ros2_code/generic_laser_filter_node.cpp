class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  rclcpp::Node::SharedPtr nh_;

  // Components for tf::MessageFilter
  tf2_ros::Buffer buffer_;
  TransformListener tf_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> tf_filter_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

  rclcpp::TimerBase::SharedPtr deprecation_timer_;

private:
  void foo(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
  }

public:
  // Constructor
  GenericLaserScanFilterNode(rclcpp::Node::SharedPtr nh)
      : nh_(nh),
        buffer_(nh_->get_clock()),
        tf_(buffer_),
        scan_sub_(nh_.get(), "scan", rclcpp::SensorDataQoS()),
        tf_filter_(scan_sub_, buffer_, "base_link", 50, *nh_),
        filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // Initialize filter chain with node parameters and interfaces
    filter_chain_.configure(nh_->get_node_parameters_interface(),
                            nh_->get_node_logging_interface(),
                            nh_->get_node_topics_interface());

    // Set message filter tolerance to 30ms
    tf_filter_.setTolerance(std::chrono::milliseconds(30));

    // Register callback for synchronized messages
    tf_filter_.registerCallback(std::bind(&GenericLaserScanFilterNode::callback, this, std::placeholders::_1));

    // Create publisher for filtered scan
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("output", rclcpp::SensorDataQoS());

    // Create recurring timer for deprecation warning every 5 seconds
    deprecation_timer_ = nh_->create_wall_timer(
        5s, [this]() {
          RCLCPP_WARN(nh_->get_logger(),
                      "This node is deprecated. Please migrate to 'scan_to_scan_filter_chain'.");
        });
  }

  // Callback
  void callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg_in)
  {
    // Run the filter chain
    filter_chain_.update(*msg_in, msg_);

    // Publish the output
    output_pub_->publish(msg_);
  }
};