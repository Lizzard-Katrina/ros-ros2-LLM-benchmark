#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <queue>
#include <memory>

struct OutgoingMessage {
    std::string topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    ros::Time time;
};

std::queue<OutgoingMessage> g_queue;

void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
             const std::string& topic)
{
    Time rectime = Time::now();
    
    if (options_.verbose)
        cout << "Received message on topic " << subscriber->getTopic() << endl;
    OutgoingMessage out;
    // TODO: push incoming message into queue
    //END
    ROS_INFO_STREAM("Message queued on topic: " << topic);
}
