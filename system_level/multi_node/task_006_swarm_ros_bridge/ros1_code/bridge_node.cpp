
#include "bridge_node.hpp"

// /* send messages frequency control */
// // this is the original freq control func that has been deprecated
// bool send_freq_control(int i)
// {
//   ros::Time t_now = ros::Time::now(); 
//   bool discard_flag;
//   if ((t_now - sub_t_last[i]).toSec() * sendTopics[i].max_freq < 1.0) {
//     discard_flag = true;
//   }
//   else {
//     discard_flag = false;
//     sub_t_last[i] = t_now; 
//   }
//   return discard_flag; // flag of discarding this message
// }

/* send messages frequency control */
bool send_freq_control(int i)
{
  bool discard_flag;
  ros::Time t_now = ros::Time::now(); 
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).toSec() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).toSec() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
// TODO: complete the below 3 functions (voids) that
// This logic must form a complete data path:
// 1. Receive ROS messages from subscribed topics
// 2. Apply frequency control if configured
// 3. Serialize ROS messages into a byte buffer
// 4. Send messages through ZeroMQ PUB sockets
// 5. Receive messages from ZeroMQ SUB sockets
// 6. Deserialize received data into ROS messages
// 7. Publish them to the corresponding ROS topics
// Ensure correctness for multiple topics, namespaces, and concurrent threads.
void sub_cb(const T &msg)
{
}
template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
}
void recv_func(int i)
{
  return;
}
// END of TODO

/* close recv socket, unsubscribe ROS topic */
void stop_send(int i)
{
  // senders[i]->unbind(std::string const &endpoint);
  senders[i]->close(); // close the send socket
  topic_subs[i].shutdown(); // unsubscribe
}

/* stop recv thread, close recv socket, unadvertise ROS topic */
void stop_recv(int i)
{
  recv_thread_flags[i] = false; // finish recv_func()
  // receivers[i]->disconnect(std::string &endpoint);
  receivers[i]->close(); // close the receive socket
  topic_pubs[i].shutdown(); // unadvertise
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;
  ns = ros::this_node::getNamespace(); // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  // get hostnames and IPs
  if (nh.getParam("IP", ip_xml) == false){
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return 1;
  }
  // get "send topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("send_topics", send_topics_xml)){
    ROS_ASSERT(send_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_send = send_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No send_topics found in the configuration!");
    len_send = 0;
  }
  // get "receive topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("recv_topics", recv_topics_xml)){
    ROS_ASSERT(recv_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_recv = recv_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No recv_topics found in the configuration!");
    len_recv = 0;
  }

  if (len_send > SUB_MAX)
  {
    ROS_FATAL("[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }

  std::cout << "-------------IP------------" << std::endl;
  for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter)
  {
    std::string host_name = iter->first;
    std::string host_ip = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end())
    { // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    ip_map[host_name] = host_ip;
  }

  std::cout << "--------send topics--------" << std::endl;
  std::set<int> srcPorts; // for duplicate check 
  for (int32_t i=0; i < len_send; ++i)
  {
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    int max_freq = send_topic_xml["max_freq"];
    std::string srcIP = ip_map[send_topic_xml["srcIP"]];
    int srcPort = send_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    int max_freq = recv_topic_xml["max_freq"];
    std::string srcIP = ip_map[recv_topic_xml["srcIP"]];
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  (from " << recv_topic_xml["srcIP"]  << ")" << std::endl;
  }

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
    sub_t_last.emplace_back(ros::Time::now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    ros::Subscriber subscriber;
    // The uniform callback function is sub_cb()
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh_public, i);
    topic_subs.emplace_back(subscriber);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    ros::Publisher publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh_public);
    topic_pubs.emplace_back(publisher);
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_thread_flags.emplace_back(true); // enable receive thread flags
    recv_flags_last.emplace_back(false); // receive success flag
    recv_threads.emplace_back(std::thread(&recv_func, i));
  }

  ros::spin();

  // ***************** stop send/receive ******************************
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  for (int32_t i=0; i < len_recv; ++i){
    stop_recv(i);
  }
  
  return 0;
}
