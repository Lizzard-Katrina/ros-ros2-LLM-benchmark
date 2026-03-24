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

        // TODO: create ApproximateTime synchronizer and register syncCallback
        // end: this is the end of the todo
    }

    void syncCallback(
        const sensor_msgs::ImageConstPtr& left,
        const sensor_msgs::ImageConstPtr& right)
    {
        ROS_INFO("Left stamp: %u, Right stamp: %u", left->header.stamp.sec, right->header.stamp.sec);
    }

private:
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> right_sub_;
    boost::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>> sync_;
};

int main(int argc, char** argv) {
    // TODO: initialize ROS node, create NodeHandle, instantiate StereoSync and spin
    // end: this is the end of the todo
    return 0;
}
