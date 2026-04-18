# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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

----------------------------


# LLM Response

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSync {
public:
    StereoSync(rclcpp::Node& node) {
        left_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            node, "/left/image", rclcpp::QoS(1)));
        right_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            node, "/right/image", rclcpp::QoS(1)));

        // TODO: create ApproximateTime synchronizer and register syncCallback
        sync_.reset(new message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
            10));
        sync_->connectInput(left_sub_, right_sub_);
        sync_->registerCallback(
            std::bind(&StereoSync::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
        // end: this is the end of the todo
    }

    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left,
        const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        RCLCPP_INFO(rclcpp::get_logger("StereoSync"), "Left stamp: %u, Right stamp: %u", left->header.stamp.sec, right->header.stamp.sec);
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
};

int main(int argc, char** argv) {
    // TODO: initialize ROS node, create NodeHandle, instantiate StereoSync and spin
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_sync");
    StereoSync stereo_sync(*node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    // end: this is the end of the todo
    return 0;
}
```