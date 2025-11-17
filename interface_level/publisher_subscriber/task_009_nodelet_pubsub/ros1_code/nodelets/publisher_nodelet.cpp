#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

namespace nodelet_pubsub {

class PublisherNodelet : public nodelet::Nodelet {
public:
    virtual void onInit() {
        ros::NodeHandle& nh = getNodeHandle();

        // TODO: define topic name and queue size
        pub_ = nh.advertise<std_msgs::String>("_____", 10);

        // publish one message as example
        std_msgs::String msg;
        msg.data = "Hello from nodelet publisher!";
        pub_.publish(msg);
    }

private:
    ros::Publisher pub_;
};

} // namespace

PLUGINLIB_EXPORT_CLASS(nodelet_pubsub::PublisherNodelet, nodelet::Nodelet)

