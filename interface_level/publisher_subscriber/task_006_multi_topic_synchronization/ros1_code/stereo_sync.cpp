#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSync {
public:
    StereoSync(ros::NodeHandle& nh) {
        left_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
            nh, "/left/image", 1));
        right_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
            nh, "/right/image", 1));

        // TODO: define ApproximateTime or ExactTime policy
        // TODO: create synchronizer
        // TODO: register callback
    }

    void syncCallback(
        const sensor_msgs::ImageConstPtr& left,
        const sensor_msgs::ImageConstPtr& right)
    {
        // TODO: handle synchronized messages
    }

private:
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> right_sub_;

    // TODO: synchronizer object
};

int main(int argc, char** argv) {
    // TODO: initialize node
    // TODO: create NodeHandle
    // TODO: instantiate StereoSync
    // TODO: spin
    return 0;
}
