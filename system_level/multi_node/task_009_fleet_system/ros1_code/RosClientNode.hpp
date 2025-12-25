#pragma once

#include <flatbuffers/flatbuffers.h>
#include <ros/ros.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <chrono>
#include <thread>
#include <unordered_map>

#include "decode_ros.hpp"
#include "encode_ros.hpp"
#include "topic_config.hpp"

class RosClientNode : public QObject {
  Q_OBJECT

  struct TopicParams {
    double priority;
    double rate_limit;
    bool no_drop;
  };

  ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
  ros::NodeHandle n;

  using TopicString = std::string;
  using MsgTypeString = std::string;
  std::vector<TopicString> pub_remote_topics;
  std::unordered_map<TopicString, ros::Subscriber> subs;
  std::unordered_map<TopicString, ros::Publisher> pubs;
  std::unordered_map<TopicString, TopicParams> topic_params;
  std::unordered_map<
      MsgTypeString, std::function<void(const QByteArray&, const TopicString&)>>
      pub_fns;

  const int verbosity_;

  /**
   * @brief Emit a ros_message_encoded() signal given a message and metadata.
   *
   * @tparam T type of msg
   * @param msg message to encode
   * @param msg_type type of the message; can be obtained using
   * ros::message_traits::DataType
   * @param from_topic topic on which message was received
   * @param to_topic remote topic to send to
   */
  template <typename T>
  void encode_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& from_topic,
      const std::string& to_topic) {

    // encode message
    flatbuffers::FlatBufferBuilder fbb;
    auto metadata = encode_metadata(fbb, msg_type, to_topic);
    auto root_offset = encode<T>(fbb, msg, metadata);
    fbb.Finish(flatbuffers::Offset<void>(root_offset));
    const QByteArray data{reinterpret_cast<const char*>(fbb.GetBufferPointer()),
                          static_cast<int>(fbb.GetSize())};
    const TopicParams& params = topic_params[from_topic];
    Q_EMIT ros_message_encoded(
        QString::fromStdString(to_topic),
        data,
        params.priority,
        params.rate_limit,
        params.no_drop);
  }

  template <typename T>
  void publish_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& to_topic) {
    pubs[msg_type].publish(msg);
  }
  /**
   * @brief Set up pub/sub for a particular message type and topic.
   *
   * Creates a subscriber to the given topic, and emits ros_message_encoded()
   * signals for each message received.
   * @tparam T the ROS message type
   * @param from_topic the topic to subscribe to
   * @param to_topic the topic to publish to the server
   * @param max_publish_rate_hz the maximum number of messages per second to
   * publish on this topic
   */

  //TODO: 
  // Implement ROS topic (3 incomplete functions) integration logic for a fleet client node.
// This logic should:
// 1. Dynamically register local ROS subscriptions and encode outgoing messages.
// 2. Register remote message handlers and publish decoded messages locally.
// 3. Bridge multiple local and remote topics while avoiding feedback loops.
// 4. Ensure correct wiring between topic configuration and runtime behavior.	
  template <typename T>
  void register_local_msg_type(
	const std::string& from_topic, const std::string& to_topic) {
  }

  template <typename T>
  void register_remote_msg_type(
      const std::string& from_topic, const std::string& to_topic) {
  }
 Q_SIGNALS:
  void ros_message_encoded(
      const QString& topic, const QByteArray& data, double priority, double rate_limit,
      bool no_drop);

 public Q_SLOTS:
  /**
   * @brief Attempt to decode an publish message data.
   *
   * Must call register_msg_type<T> before a message of type T can be decoded.
   * @param data the Flatbuffer-encoded message data
   */
  void decode_net_message(const QByteArray& data) {
    // extract metadata
    const fb::MsgWithMetadata* msg =
        flatbuffers::GetRoot<fb::MsgWithMetadata>(data.data());
    const std::string& msg_type = msg->__metadata()->type()->str();
    const std::string& topic = msg->__metadata()->topic()->str();

    // try to publish
    if (pub_fns.count(msg_type) == 0) {
      // if you get this unexpectedly, ensure that:
      // 1) you have registered the message type you intend to receive in your
      // configuration, and 2) you are sending a valid, fully-qualified message
      // type name (if you manually construct the flatbuffer)
      if (verbosity_ > 0) {
        std::cerr << "ignoring message of unregistered type " << msg_type
                  << std::endl;
      }
      return;
    }

    if (verbosity_ > 1) {
      std::cout << "Received message of type " << msg_type << " on topic " << topic << std::endl;
    }

    pub_fns[msg_type](data, topic);
  }

 public:
  /**
   * @brief subscribe to remote messages that were specified in the config by
   * calling register_remote_msg_type. This function should run once a websocket
   * connection has been established
   */
  void subscribe_remote_msgs() {
  }
// END OF TODO

  /**
   * @brief Register new listeners based on the given topic configuration.
   */
  template <typename Config>
  void configure(const Config& config);

  template <typename T>
  void configure(const topic_config::SendLocalTopic<T>& config) {
    config.assert_valid();
    register_local_msg_type<T>(config.from, config.to);
    const std::string full_from_topic = ros::names::resolve(config.from);
    topic_params[full_from_topic] =
        TopicParams{config.priority, config.rate_limit_hz.is_set() ? config.rate_limit_hz : 0, config.no_drop};
  }

  template <typename T>
  void configure(const topic_config::ReceiveRemoteTopic<T>& config) {
    config.assert_valid();
    register_remote_msg_type<T>(config.from, config.to);
  }

  RosClientNode(int verbosity) : verbosity_(verbosity)  {
    // run forever
    spinner.start();
    if (verbosity_ > 0) {
      std::cout << "Started ROS Node" << std::endl;
    }
  }
};
