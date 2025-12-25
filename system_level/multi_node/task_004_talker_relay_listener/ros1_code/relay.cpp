#include "ros/ros.h"
#include "std_msgs/String.h"

class RelayNode
{
public:
  RelayNode()
  {
    // TODO:
    // 1. Initialize a ROS NodeHandle
    // 2. Create a subscriber that listens to the talker topic (std_msgs/String)
    // 3. Create a publisher that republishes messages to a new topic
    // 4. Register a callback that forwards incoming messages unchanged
  }

private:
  // TODO:
  // - ros::NodeHandle
  // - ros::Subscriber
  // - ros::Publisher

  void relayCallback(const std_msgs::String::ConstPtr& msg)
  {
    // TODO:
    // Republish the received message to the output topic
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relay");

  // TODO:
  // Instantiate RelayNode and spin

  return 0;
}
