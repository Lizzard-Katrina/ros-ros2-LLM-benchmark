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
 * Core Idea: It would create the receving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode() : Node("swarm_bridge")
  {
    // Initialize variables
    sub_t_last_.resize(SUB_MAX);
    send_num_.resize(SUB_MAX);
    recv_thread_flags_.resize(SUB_MAX);
    recv_flags_last_.resize(SUB_MAX);
    recv_threads_.resize(SUB_MAX);
    topic_subs_.resize(SUB_MAX);
    topic_pubs_.resize(SUB_MAX);
    senders_.resize(SUB_MAX);
    receivers_.resize(SUB_MAX);

    // Get parameters
    this->get_parameter("IP", ip_xml_);
    this->get_parameter("send_topics", send_topics_xml_);
    this->get_parameter("recv_topics", recv_topics_xml_);

    // Initialize send and receive topics
    len_send_ = send_topics_xml_.size();
    len_recv_ = recv_topics_xml_.size();

    // Check for duplicate ports
    std::set<int> srcPorts;
    for (int32_t i = 0; i < len_send_; ++i)
    {
      ROS_ASSERT(send_topics_xml_[i].get_type() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml_[i];
      std::string topic_name = send_topic_xml["topic_name"];
      std::string msg_type = send_topic_xml["msg_type"];
      int max_freq = send_topic_xml["max_freq"];
      std::string srcIP = ip_map_[send_topic_xml["srcIP"]];
      int srcPort = send_topic_xml["srcPort"];
      TopicInfo topic = {.name = topic_name, .type = msg_type, .max_freq = max_freq, .ip = srcIP, .port = srcPort};
      sendTopics_.emplace_back(topic);
      if (srcPorts.find(srcPort) != srcPorts.end())
      {
        RCLCPP_FATAL(this->get_logger(), "[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
        return;
      }
      srcPorts.insert(srcPort);
    }

    // Initialize receive topics
    for (int32_t i = 0; i < len_recv_; ++i)
    {
      ROS_ASSERT(recv_topics_xml_[i].get_type() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml_[i];
      std::string topic_name = recv_topic_xml["topic_name"];
      std::string msg_type = recv_topic_xml["msg_type"];
      int max_freq = recv_topic_xml["max_freq"];
      std::string srcIP = ip_map_[recv_topic_xml["srcIP"]];
      int srcPort = recv_topic_xml["srcPort"];
      TopicInfo topic = {.name = topic_name, .type = msg_type, .max_freq = max_freq, .ip = srcIP, .port = srcPort};
      recvTopics_.emplace_back(topic);
    }

    // Initialize zmq sockets
    for (int32_t i = 0; i < len_send_; ++i)
    {
      const std::string url = "tcp://" + sendTopics_[i].ip + ":" + std::to_string(sendTopics_[i].port);
      std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context_, zmqpp::socket_type::pub));
      sender->bind(url);
      senders_.emplace_back(std::move(sender));
    }

    for (int32_t i = 0; i < len_recv_; ++i)
    {
      const std::string url = "tcp://" + recvTopics_[i].ip + ":" + std::to_string(recvTopics_[i].port);
      std::string const zmq_topic = ""; // "" means all zmq topic
      std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context_, zmqpp::socket_type::sub));
      receiver->subscribe(zmq_topic);
      receiver->connect(url);
      receivers_.emplace_back(std::move(receiver));
    }

    // Initialize ROS subscribers and publishers
    for (int32_t i = 0; i < len_send_; ++i)
    {
      sub_t_last_[i] = this->get_clock()->now();
      send_num_[i] = 0;
      topic_subs_[i] = this->create_subscription<T>(sendTopics_[i].name, 10, std::bind(&BridgeNode::sub_cb, this, std::placeholders::_1, i));
    }

    for (int32_t i = 0; i < len_recv_; ++i)
    {
      topic_pubs_[i] = this->create_publisher<T>(recvTopics_[i].name, 10);
    }

    // Launch receive threads
    for (int32_t i = 0; i < len_recv_; ++i)
    {
      recv_thread_flags_[i] = true;
      recv_flags_last_[i] = false;
      recv_threads_[i] = std::thread(&BridgeNode::recv_func, this, i);
    }
  }

  ~BridgeNode()
  {
    // Stop send and receive threads
    for (int32_t i = 0; i < len_send_; ++i)
    {
      stop_send(i);
    }

    for (int32_t i = 0; i < len_recv_; ++i)
    {
      stop_recv(i);
    }
  }

  /* send messages frequency control */
  bool send_freq_control(int i)
  {
    bool discard_flag;
    auto t_now = this->get_clock()->now();
    if ((send_num_[i] + 1) / (t_now - sub_t_last_[i]).seconds() > sendTopics_[i].max_freq)
    {
      discard_flag = true;
    }
    else
    {
      discard_flag = false;
      send_num_[i]++;
    }
    if ((t_now - sub_t_last_[i]).seconds() > 1.0)
    {
      sub_t_last_[i] = t_now;
      send_num_[i] = 0;
    }
    return discard_flag;
  }

  /* uniform callback functions for ROS subscribers */
  template <typename T>
  void sub_cb(const T &msg, int i)
  {
    /* frequency control */
    auto ignore_flag = send_freq_control(i);
    if (ignore_flag)
    {
      return;
    }

    // Serialize the message
    eprosima::fastcdr::FastBuffer fast_buffer;
    eprosima::fastcdr::Cdr ser(fast_buffer);
    msg.serialize(ser);
    zmqpp::message zmq_msg;
    zmq_msg << fast_buffer.get_data() << fast_buffer.get_size();
    senders_[i]->send(zmq_msg);
  }

  /* uniform deserialize and publish the receiving messages */
  template <typename T>
  void deserialize_pub(uint8_t *buffer_ptr, size_t msg_size, int i)
  {
    T msg;
    eprosima::fastcdr::FastBuffer fast_buffer(buffer_ptr, msg_size);
    eprosima::fastcdr::Cdr des(fast_buffer);
    msg.deserialize(des);
    topic_pubs_[i]->publish(msg);
  }

  /* receive thread function to receive messages and publish them */
  void recv_func(int i)
  {
    while (recv_thread_flags_[i])
    {
      /* receive and process message */
      zmqpp::message recv_array;
      bool recv_flag;
      bool dont_block = false;
      if (recv_flag = receivers_[i]->receive(recv_array, dont_block))
      {
        size_t data_len;
        recv_array >> data_len;
        std::unique_ptr<uint8_t> recv_buffer(new uint8_t[data_len]);
        memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
        deserialize_pub<T>(recv_buffer.get(), data_len, i);
      }

      if (dont_block)
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      else
      {
        if (recv_flag != recv_flags_last_[i])
        {
          RCLCPP_INFO(this->get_logger(), "[bridge node] \"%s\" received!", recvTopics_[i].name.c_str());
        }
        recv_flags_last_[i] = recv_flag;
      }
    }
  }

  /* close recv socket, unsubscribe ROS topic */
  void stop_send(int i)
  {
    senders_[i]->close();
    topic_subs_[i].reset();
  }

  /* stop recv thread, close recv socket, unadvertise ROS topic */
  void stop_recv(int i)
  {
    recv_thread_flags_[i] = false;
    receivers_[i]->close();
    topic_pubs_[i].reset();
  }

private:
  std::vector<rclcpp::Time> sub_t_last_;
  std::vector<int> send_num_;
  std::vector<bool> recv_thread_flags_;
  std::vector<bool> recv_flags_last_;
  std::vector<std::thread> recv_threads_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_subs_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> topic_pubs_;
  std::vector<std::unique_ptr<zmqpp::socket>> senders_;
  std::vector<std::unique_ptr<zmqpp::socket>> receivers_;
  zmqpp::context context_;
  XmlRpc::XmlRpcValue ip_xml_;
  XmlRpc::XmlRpcValue send_topics_xml_;
  XmlRpc::XmlRpcValue recv_topics_xml_;
  std::map<std::string, std::string> ip_map_;
  std::vector<TopicInfo> sendTopics_;
  std::vector<TopicInfo> recvTopics_;
  int len_send_;
  int len_recv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}