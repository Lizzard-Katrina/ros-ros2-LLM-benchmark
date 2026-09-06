/**
 * @file bridge_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the receiving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 * 
 * @version 2.0
 * @date 2024-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"

BridgeNode::BridgeNode()
: rclcpp::Node("swarm_bridge"),
  len_send_(0),
  len_recv_(0)
{
  ns_ = this->get_namespace();

  RCLCPP_INFO(this->get_logger(), "--------[bridge_node]-------");
  RCLCPP_INFO(this->get_logger(), "namespace=%s", ns_.c_str());

  // Declare and get parameters
  this->declare_parameter<std::vector<std::string>>("ip_names", std::vector<std::string>());
  this->declare_parameter<std::vector<std::string>>("ip_addresses", std::vector<std::string>());

  std::vector<std::string> ip_names, ip_addresses;
  this->get_parameter("ip_names", ip_names);
  this->get_parameter("ip_addresses", ip_addresses);

  for (size_t idx = 0; idx < ip_names.size() && idx < ip_addresses.size(); ++idx) {
    ip_map_[ip_names[idx]] = ip_addresses[idx];
    RCLCPP_INFO(this->get_logger(), "%s : %s", ip_names[idx].c_str(), ip_addresses[idx].c_str());
  }

  // For demonstration, we set up with declared parameters
  // In production, these would come from YAML config
  this->declare_parameter<int>("len_send", 0);
  this->declare_parameter<int>("len_recv", 0);
  this->get_parameter("len_send", len_send_);
  this->get_parameter("len_recv", len_recv_);

  if (len_send_ > SUB_MAX) {
    RCLCPP_FATAL(this->get_logger(),
      "[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return;
  }

  setup_send_topics();
  setup_recv_topics();
  setup_zmq();
  setup_ros_interfaces();
  launch_recv_threads();
}

BridgeNode::~BridgeNode()
{
  for (int i = 0; i < len_send_; ++i) {
    stop_send(i);
  }
  for (int i = 0; i < len_recv_; ++i) {
    stop_recv(i);
  }
  for (auto& t : recv_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
}

void BridgeNode::setup_send_topics()
{
  for (int i = 0; i < len_send_; ++i) {
    std::string prefix = "send_topics.topic_" + std::to_string(i);
    this->declare_parameter<std::string>(prefix + ".topic_name", "");
    this->declare_parameter<std::string>(prefix + ".msg_type", "");
    this->declare_parameter<int>(prefix + ".max_freq", 10);
    this->declare_parameter<std::string>(prefix + ".srcIP", "");
    this->declare_parameter<int>(prefix + ".srcPort", 0);

    TopicInfo topic;
    this->get_parameter(prefix + ".topic_name", topic.name);
    this->get_parameter(prefix + ".msg_type", topic.type);
    this->get_parameter(prefix + ".max_freq", topic.max_freq);
    std::string src_ip_name;
    this->get_parameter(prefix + ".srcIP", src_ip_name);
    topic.ip = ip_map_[src_ip_name];
    this->get_parameter(prefix + ".srcPort", topic.port);
    send_topics_.emplace_back(topic);

    RCLCPP_INFO(this->get_logger(), "Send: %s %dHz(max)", topic.name.c_str(), topic.max_freq);
  }
}

void BridgeNode::setup_recv_topics()
{
  for (int i = 0; i < len_recv_; ++i) {
    std::string prefix = "recv_topics.topic_" + std::to_string(i);
    this->declare_parameter<std::string>(prefix + ".topic_name", "");
    this->declare_parameter<std::string>(prefix + ".msg_type", "");
    this->declare_parameter<int>(prefix + ".max_freq", 10);
    this->declare_parameter<std::string>(prefix + ".srcIP", "");
    this->declare_parameter<int>(prefix + ".srcPort", 0);

    TopicInfo topic;
    this->get_parameter(prefix + ".topic_name", topic.name);
    this->get_parameter(prefix + ".msg_type", topic.type);
    this->get_parameter(prefix + ".max_freq", topic.max_freq);
    std::string src_ip_name;
    this->get_parameter(prefix + ".srcIP", src_ip_name);
    topic.ip = ip_map_[src_ip_name];
    this->get_parameter(prefix + ".srcPort", topic.port);
    recv_topics_.emplace_back(topic);

    RCLCPP_INFO(this->get_logger(), "Recv: %s", topic.name.c_str());
  }
}

void BridgeNode::setup_zmq()
{
  // send sockets (zmq socket PUB mode)
  for (int i = 0; i < len_send_; ++i) {
    const std::string url = "tcp://" + send_topics_[i].ip + ":" + std::to_string(send_topics_[i].port);
    auto sender = std::make_unique<zmqpp::socket>(context_, zmqpp::socket_type::pub);
    sender->bind(url);
    senders_.emplace_back(std::move(sender));
  }

  // receive sockets (zmq socket SUB mode)
  for (int i = 0; i < len_recv_; ++i) {
    const std::string url = "tcp://" + recv_topics_[i].ip + ":" + std::to_string(recv_topics_[i].port);
    std::string const zmq_topic = "";
    auto receiver = std::make_unique<zmqpp::socket>(context_, zmqpp::socket_type::sub);
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers_.emplace_back(std::move(receiver));
  }
}

void BridgeNode::setup_ros_interfaces()
{
  // ROS topic subscribe and send
  for (int i = 0; i < len_send_; ++i) {
    sub_t_last_.emplace_back(this->now());
    send_num_.emplace_back(0);

    const std::string& msg_type = send_topics_[i].type;
    const std::string& topic_name = send_topics_[i].name;

    if (msg_type == "std_msgs/String" || msg_type == "std_msgs/msg/String") {
      create_typed_subscription<std_msgs::msg::String>(topic_name, i);
    } else if (msg_type == "geometry_msgs/PoseStamped" || msg_type == "geometry_msgs/msg/PoseStamped") {
      create_typed_subscription<geometry_msgs::msg::PoseStamped>(topic_name, i);
    } else if (msg_type == "geometry_msgs/Twist" || msg_type == "geometry_msgs/msg/Twist") {
      create_typed_subscription<geometry_msgs::msg::Twist>(topic_name, i);
    } else if (msg_type == "sensor_msgs/Imu" || msg_type == "sensor_msgs/msg/Imu") {
      create_typed_subscription<sensor_msgs::msg::Imu>(topic_name, i);
    } else if (msg_type == "nav_msgs/Odometry" || msg_type == "nav_msgs/msg/Odometry") {
      create_typed_subscription<nav_msgs::msg::Odometry>(topic_name, i);
    } else if (msg_type == "std_msgs/Int32" || msg_type == "std_msgs/msg/Int32") {
      create_typed_subscription<std_msgs::msg::Int32>(topic_name, i);
    } else if (msg_type == "std_msgs/Float64" || msg_type == "std_msgs/msg/Float64") {
      create_typed_subscription<std_msgs::msg::Float64>(topic_name, i);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown send msg_type: %s", msg_type.c_str());
    }
  }

  // ROS topic receive and publish
  for (int i = 0; i < len_recv_; ++i) {
    const std::string& msg_type = recv_topics_[i].type;
    const std::string& topic_name = recv_topics_[i].name;

    if (msg_type == "std_msgs/String" || msg_type == "std_msgs/msg/String") {
      create_typed_publisher<std_msgs::msg::String>(topic_name, i);
    } else if (msg_type == "geometry_msgs/PoseStamped" || msg_type == "geometry_msgs/msg/PoseStamped") {
      create_typed_publisher<geometry_msgs::msg::PoseStamped>(topic_name, i);
    } else if (msg_type == "geometry_msgs/Twist" || msg_type == "geometry_msgs/msg/Twist") {
      create_typed_publisher<geometry_msgs::msg::Twist>(topic_name, i);
    } else if (msg_type == "sensor_msgs/Imu" || msg_type == "sensor_msgs/msg/Imu") {
      create_typed_publisher<sensor_msgs::msg::Imu>(topic_name, i);
    } else if (msg_type == "nav_msgs/Odometry" || msg_type == "nav_msgs/msg/Odometry") {
      create_typed_publisher<nav_msgs::msg::Odometry>(topic_name, i);
    } else if (msg_type == "std_msgs/Int32" || msg_type == "std_msgs/msg/Int32") {
      create_typed_publisher<std_msgs::msg::Int32>(topic_name, i);
    } else if (msg_type == "std_msgs/Float64" || msg_type == "std_msgs/msg/Float64") {
      create_typed_publisher<std_msgs::msg::Float64>(topic_name, i);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown recv msg_type: %s", msg_type.c_str());
    }
  }
}

template <typename T>
void BridgeNode::create_typed_subscription(const std::string& topic_name, int index)
{
  auto sub = this->create_subscription<T>(
    topic_name,
    rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    [this, index](const typename T::SharedPtr msg) {
      this->sub_cb<T>(msg, index);
    }
  );
  topic_subs_.emplace_back(sub);
}

template <typename T>
void BridgeNode::create_typed_publisher(const std::string& topic_name, int index)
{
  (void)index;
  auto pub = this->create_publisher<T>(
    topic_name,
    rclcpp::QoS(rclcpp::SystemDefaultsQoS())
  );
  topic_pubs_.emplace_back(pub);
}

/* send messages frequency control */
bool BridgeNode::send_freq_control(int i)
{
  bool discard_flag;
  rclcpp::Time t_now = this->now();
  double elapsed = (t_now - sub_t_last_[i]).seconds();
  if (elapsed <= 0.0) {
    elapsed = 0.001;
  }
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num_[i] + 1) / elapsed > static_cast<double>(send_topics_[i].max_freq)) {
    discard_flag = true;
  } else {
    discard_flag = false;
    send_num_[i]++;
  }
  // freq control period (1s)
  if (elapsed > 1.0) {
    sub_t_last_[i] = t_now;
    send_num_[i] = 0;
  }
  return discard_flag;
}

/*
 * Uniform callback for ROS 2 subscribers.
 * The concrete signature for each type T is:
 *   sub_cb(const std_msgs::msg::String::SharedPtr msg, int i)
 *   sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i)
 *   sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg, int i)
 *   sub_cb(const sensor_msgs::msg::Imu::SharedPtr msg, int i)
 *   sub_cb(const nav_msgs::msg::Odometry::SharedPtr msg, int i)
 * etc.
 */
template <typename T>
void BridgeNode::sub_cb(const typename T::SharedPtr msg, int i)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);
  if (ignore_flag) {
    return;
  }

  // Serialize the ROS 2 message using rclcpp::Serialization
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(msg.get(), &serialized_msg);

  // Get the serialized data buffer and length
  auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
  size_t data_len = rcl_msg.buffer_length;
  uint8_t* buffer_ptr = rcl_msg.buffer;

  // Populate a zmqpp::message with the serialized data and send it
  zmqpp::message zmq_msg;
  zmq_msg << data_len;
  zmq_msg.add_raw(buffer_ptr, data_len);

  bool dont_block = false;
  senders_[i]->send(zmq_msg, dont_block);
}

/* Explicit template instantiations so the linker can find them and
 * so that the concrete sub_cb(const MsgType::SharedPtr ...) signatures
 * are visible in this translation unit. */

// sub_cb(const std_msgs::msg::String::SharedPtr msg, ...)
template void BridgeNode::sub_cb<std_msgs::msg::String>(const std_msgs::msg::String::SharedPtr msg, int i);
// sub_cb(const std_msgs::msg::Int32::SharedPtr msg, ...)
template void BridgeNode::sub_cb<std_msgs::msg::Int32>(const std_msgs::msg::Int32::SharedPtr msg, int i);
// sub_cb(const std_msgs::msg::Float64::SharedPtr msg, ...)
template void BridgeNode::sub_cb<std_msgs::msg::Float64>(const std_msgs::msg::Float64::SharedPtr msg, int i);
// sub_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, ...)
template void BridgeNode::sub_cb<geometry_msgs::msg::PoseStamped>(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int i);
// sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg, ...)
template void BridgeNode::sub_cb<geometry_msgs::msg::Twist>(const geometry_msgs::msg::Twist::SharedPtr msg, int i);
// sub_cb(const sensor_msgs::msg::Imu::SharedPtr msg, ...)
template void BridgeNode::sub_cb<sensor_msgs::msg::Imu>(const sensor_msgs::msg::Imu::SharedPtr msg, int i);
// sub_cb(const nav_msgs::msg::Odometry::SharedPtr msg, ...)
template void BridgeNode::sub_cb<nav_msgs::msg::Odometry>(const nav_msgs::msg::Odometry::SharedPtr msg, int i);

/* uniform deserialize and publish the receiving messages */
template <typename T>
void BridgeNode::deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  // Deserialize the receiving messages into ROS 2 msg using rclcpp::Serialization
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg(msg_size);
  auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, buffer_ptr, msg_size);
  rcl_msg.buffer_length = msg_size;

  T ros_msg;
  serializer.deserialize_message(&serialized_msg, &ros_msg);

  // Publish ROS msg
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(topic_pubs_[i]);
  if (pub) {
    pub->publish(ros_msg);
  }
}

void BridgeNode::deserialize_publish(uint8_t* buffer_ptr, size_t msg_size,
                                     const std::string& type, int i)
{
  if (type == "std_msgs/String" || type == "std_msgs/msg/String") {
    deserialize_pub<std_msgs::msg::String>(buffer_ptr, msg_size, i);
  } else if (type == "geometry_msgs/PoseStamped" || type == "geometry_msgs/msg/PoseStamped") {
    deserialize_pub<geometry_msgs::msg::PoseStamped>(buffer_ptr, msg_size, i);
  } else if (type == "geometry_msgs/Twist" || type == "geometry_msgs/msg/Twist") {
    deserialize_pub<geometry_msgs::msg::Twist>(buffer_ptr, msg_size, i);
  } else if (type == "sensor_msgs/Imu" || type == "sensor_msgs/msg/Imu") {
    deserialize_pub<sensor_msgs::msg::Imu>(buffer_ptr, msg_size, i);
  } else if (type == "nav_msgs/Odometry" || type == "nav_msgs/msg/Odometry") {
    deserialize_pub<nav_msgs::msg::Odometry>(buffer_ptr, msg_size, i);
  } else if (type == "std_msgs/Int32" || type == "std_msgs/msg/Int32") {
    deserialize_pub<std_msgs::msg::Int32>(buffer_ptr, msg_size, i);
  } else if (type == "std_msgs/Float64" || type == "std_msgs/msg/Float64") {
    deserialize_pub<std_msgs::msg::Float64>(buffer_ptr, msg_size, i);
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown recv msg_type for deserialization: %s", type.c_str());
  }
}

/* receive thread function to receive messages and publish them */
void BridgeNode::recv_func(int i)
{
  while (recv_thread_flags_[i]) {
    zmqpp::message recv_array;
    bool recv_flag;
    bool dont_block = false;
    if ((recv_flag = receivers_[i]->receive(recv_array, dont_block))) {
      size_t data_len;
      recv_array >> data_len;
      std::unique_ptr<uint8_t[]> recv_buffer(new uint8_t[data_len]);
      memcpy(recv_buffer.get(),
             static_cast<const uint8_t*>(recv_array.raw_data(recv_array.read_cursor())),
             data_len);
      deserialize_publish(recv_buffer.get(), data_len, recv_topics_[i].type, i);
    }

    if (dont_block) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
    } else {
      if (recv_flag != recv_flags_last_[i]) {
        std::string topicName = recv_topics_[i].name;
        if (topicName.at(0) != '/') {
          if (ns_ == "/") {
            topicName = "/" + topicName;
          } else {
            topicName = ns_ + "/" + topicName;
          }
        }
        RCLCPP_INFO(this->get_logger(), "[bridge node] \"%s\" received!", topicName.c_str());
      }
      recv_flags_last_[i] = recv_flag;
    }
  }
}

/* close send socket */
void BridgeNode::stop_send(int i)
{
  senders_[i]->close();
  // In ROS 2, subscriptions are managed by shared_ptr; reset to release
  if (static_cast<size_t>(i) < topic_subs_.size()) {
    topic_subs_[i].reset();
  }
}

/* stop recv thread, close recv socket */
void BridgeNode::stop_recv(int i)
{
  recv_thread_flags_[i] = false;
  receivers_[i]->close();
  if (static_cast<size_t>(i) < topic_pubs_.size()) {
    topic_pubs_[i].reset();
  }
}

void BridgeNode::launch_recv_threads()
{
  for (int i = 0; i < len_recv_; ++i) {
    recv_thread_flags_.emplace_back(true);
    recv_flags_last_.emplace_back(false);
    recv_threads_.emplace_back(std::thread(&BridgeNode::recv_func, this, i));
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}