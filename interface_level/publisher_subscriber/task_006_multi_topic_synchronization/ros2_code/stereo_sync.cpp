#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

class StereoSync {
public:
    StereoSync(rclcpp::Node::SharedPtr nh) : node_(nh) {
        left_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>());
        right_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>());

        left_sub_->subscribe(node_.get(), "/left/image", rmw_qos_profile_sensor_data);
        right_sub_->subscribe(node_.get(), "/right/image", rmw_qos_profile_sensor_data);

        // TODO: create ApproximateTime synchronizer and register syncCallback
        sync_.reset(new message_filters::Synchronizer<
                    message_filters::sync_policies::ApproximateTime<
                        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image>(10),
            *left_sub_, *right_sub_));

        sync_->registerCallback(
            std::bind(&StereoSync::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
        // end: this is the end of the todo
    }

    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left,
        const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Left stamp: %d, Right stamp: %d",
            left->header.stamp.sec,
            right->header.stamp.sec);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
};

int main(int argc, char** argv) {
    // TODO: initialize ROS node, create NodeHandle, instantiate StereoSync and spin
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stereo_sync");
    StereoSync stereo_sync(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    // end: this is the end of the todo
    return 0;
}