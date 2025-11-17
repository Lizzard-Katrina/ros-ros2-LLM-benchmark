#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

namespace nodelet_pubsub {

class SubscriberNodelet : public nodelet::Nodelet {
public:
    virtual void onInit() {
        ros::NodeHandle& nh = getNodeHandle();

        // TODO: define topic name
        sub_ = nh.subscribe("_____", 10, &SubscriberNodelet::callback, this);
    }

private:
    ros::Subscriber sub_;

    void callback(const std_msgs::String::ConstPtr& msg) {
        NODELET_INFO("Received: %s", msg->data.c_str());
    }
};

} // namespace

PLUGINLIB_EXPORT_CLASS(nodelet_pubsub::SubscriberNodelet, nodelet::Nodelet)
