/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 2.0
 * @date 2024-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <set>
#include <map>
#include <zmqpp/zmqpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "ros_sub_pub.hpp"

struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  std::string ip;
  int port;
};

#define SUB_MAX 100

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode();
  ~BridgeNode();

private:
  // ******************* ROS 2 interfaces ***************************
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subs_;
  std::vector<rclcpp::PublisherBase::SharedPtr> topic_pubs_;

  // ******************* ZMQ interfaces ***************************
  zmqpp::context context_;
  std::vector<std::unique_ptr<zmqpp::socket>> senders_;
  std::vector<std::unique_ptr<zmqpp::socket>> receivers_;

  // ******************* Topic configuration ***************************
  std::vector<TopicInfo> send_topics_;
  std::vector<TopicInfo> recv_topics_;
  int len_send_;
  int len_recv_;
  std::string ns_;
  std::map<std::string, std::string> ip_map_;

  // ******************* send frequency control ***************************
  std::vector<rclcpp::Time> sub_t_last_;
  std::vector<int> send_num_;
  bool send_freq_control(int i);

  // ****************** launch receive threads *****************************
  std::vector<bool> recv_thread_flags_;
  std::vector<bool> recv_flags_last_;
  std::vector<std::thread> recv_threads_;
  void recv_func(int i);

  // ***************** stop send/receive ******************************
  void stop_send(int i);
  void stop_recv(int i);

  // ***************** callback and serialization ******************************
  // ROS 2 callback: sub_cb(const MsgType::SharedPtr msg, int i)
  template <typename T>
  void sub_cb(const typename T::SharedPtr msg, int i);

  template <typename T>
  void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i);

  void deserialize_publish(uint8_t* buffer_ptr, size_t msg_size,
                           const std::string& type, int i);

  // ***************** setup helpers ******************************
  void setup_send_topics();
  void setup_recv_topics();
  void setup_zmq();
  void setup_ros_interfaces();
  void launch_recv_threads();

  template <typename T>
  void create_typed_subscription(const std::string& topic_name, int index);

  template <typename T>
  void create_typed_publisher(const std::string& topic_name, int index);
};

#endif