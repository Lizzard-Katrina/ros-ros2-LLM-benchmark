#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSync {
public:
    StereoSync(rclcpp::Node::SharedPtr node) : node_(node) {
        left_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            node_, "/left/image"));
        right_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            node_, "/right/image"));

        // Create ApproximateTime synchronizer and register syncCallback
        sync_.reset(new message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(10),
            *left_sub_, *right_sub_));
        sync_->registerCallback(std::bind(&StereoSync::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left,
        const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        RCLCPP_INFO(node_->get_logger(), "Left stamp: %u, Right stamp: %u",
            left->header.stamp.sec, right->header.stamp.sec);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stereo_sync_node");
    auto stereo_sync = std::make_shared<StereoSync>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}