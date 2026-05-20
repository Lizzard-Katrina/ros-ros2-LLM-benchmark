# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: stereo_sync.cpp
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

[FILENAME: stereo_sync.cpp]
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSync : public rclcpp::Node {
public:
    StereoSync() : Node("stereo_sync") {
        left_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            this, "/left/image"));
        right_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
            this, "/right/image"));

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), *left_sub_, *right_sub_));
        sync_->registerCallback(std::bind(&StereoSync::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left,
        const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        RCLCPP_INFO(this->get_logger(), "Left stamp: %d, Right stamp: %d", left->header.stamp.sec, right->header.stamp.sec);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}