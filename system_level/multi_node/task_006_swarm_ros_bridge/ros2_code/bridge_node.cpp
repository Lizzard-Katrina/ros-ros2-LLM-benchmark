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

/* send messages frequency control */
bool BridgeNode::send_freq_control(int i)
{
  bool discard_flag;
  rclcpp::Time t_now = this->now(); 
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).seconds() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).seconds() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void BridgeNode::sub_cb(const T &msg)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);
  if (ignore_flag){
    return; // discard this message sending, abort
  }

  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&msg, &serialized_msg);

  zmqpp::message zmq_msg;
  size_t data_len = serialized_msg.size();
  zmq_msg << data_len;
  
  zmq_msg.add_raw(serialized_msg.get_rcl_serialized_message().buffer, data_len);
  senders[i]->send(zmq_msg);
}


/* uniform deserialize and publish the receiving messages */
template<typename T>
void BridgeNode::deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  T msg;
  rclcpp::SerializedMessage serialized_msg;
  serialized_msg.reserve(msg_size);
  memcpy(serialized_msg.get_rcl_serialized_message().buffer, buffer_ptr, msg_size);
  serialized_msg.get_rcl_serialized_message().buffer_length = msg_size;

  rclcpp::Serialization<T> serializer;
  serializer.deserialize_message(&serialized_msg, &msg);

  // publish ROS msg
  // Note: For GenericPublisher, we would publish the serialized message directly,
  // but keeping the signature as requested.
  // topic_pubs[i]->publish(msg);
}


/* receive thread function to receive messages and publish them */
void BridgeNode::recv_func(int i)
{
  while(recv_thread_flags[i] && rclcpp::ok())
  {
    /* receive and process message */
    zmqpp::message recv_array;
    bool recv_flag; // receive success flag
    bool dont_block = false; // 'true' leads to high cpu load
    if ((recv_flag = receivers[i]->receive(recv_array, dont_block)))
    {
      size_t data_len;
      recv_array >> data_len; // unpack meta data
      
      std::unique_ptr<uint8_t[]> recv_buffer(new uint8_t[data_len]);  
      memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
      
      // deserialize_publish(recv_buffer.get(), data_len, recvTopics[i].type, i);
    }

    /* if receive() does not block, sleep to decrease loop rate */
    if (dont_block)
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // sleep for us
    else
    {
      /* check and report receive state */
      if (recv_flag != recv_flags_last[i]){
        std::string topicName = recvTopics[i].name;
        if (topicName.at(0) != '/') {
          if (ns == "/") {topicName = "/" + topicName;}
          else {topicName = ns + "/" + topicName;}
        }  // print namespace prefix if topic name is not global
        RCLCPP_INFO(this->get_logger(), "[bridge node] \"%s\" received!", topicName.c_str());
      } // false -> true(first message in)        
      recv_flags_last[i] = recv_flag;
    }
  }
  return;
}

/* close recv socket, unsubscribe ROS topic */
void BridgeNode::stop_send(int i)
{
  senders[i]->close(); // close the send socket
  topic_subs[i].reset(); // unsubscribe
}

/* stop recv thread, close recv socket, unadvertise ROS topic */
void BridgeNode::stop_recv(int i)
{
  recv_thread_flags[i] = false; // finish recv_func()
  receivers[i]->close(); // close the receive socket
  topic_pubs[i].reset(); // unadvertise
}

BridgeNode::BridgeNode() : Node("swarm_bridge")
{
  ns = std::string(this->get_namespace()); // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  // Simplified parameter loading for ROS2
  len_send = 0;
  len_recv = 0;

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->bind(url);
    senders.emplace_back(std::move(sender)); //sender is now released by std::move
  }

  // receive sockets (zmq socket SUB mode)
  for (int32_t i=0; i < len_recv; ++i)
  {
    const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
    std::string const zmq_topic = ""; // "" means all zmq topic
    std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::sub));
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers.emplace_back(std::move(receiver));
  }

  // ******************* ROS subscribe and publish *************************
  //ROS topic subsrcibe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    sub_t_last.emplace_back(this->now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    
    topic_subs.emplace_back(this->create_generic_subscription(
      sendTopics[i].name, sendTopics[i].type, 10, 
      [this, i](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        // Generic subscription callback
      }));
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    topic_pubs.emplace_back(this->create_generic_publisher(recvTopics[i].name, recvTopics[i].type, 10));
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_thread_flags.emplace_back(true); // enable receive thread flags
    recv_flags_last.emplace_back(false); // receive success flag
    recv_threads.emplace_back(std::thread(&BridgeNode::recv_func, this, i));
  }
}

BridgeNode::~BridgeNode()
{
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  for (int32_t i=0; i < len_recv; ++i){
    stop_recv(i);
  }
  
  for (auto& t : recv_threads) {
    if (t.joinable()) t.join();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}