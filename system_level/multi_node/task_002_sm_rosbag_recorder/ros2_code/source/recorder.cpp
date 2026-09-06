/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

// ROS 2 Migration of rosbag recorder
// This file demonstrates the ROS 2 equivalent patterns for the original
// ROS 1 rosbag recorder.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rmw/serialized_message.h>

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <sstream>
#include <functional>

namespace rosbag2_recorder {

struct OutgoingMessage {
    std::string topic;
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
    rclcpp::Time time;
    std::string topic_type;

    OutgoingMessage(const std::string& _topic,
                    std::shared_ptr<rclcpp::SerializedMessage> _msg,
                    rclcpp::Time _time,
                    const std::string& _topic_type)
        : topic(_topic), serialized_msg(_msg), time(_time), topic_type(_topic_type)
    {}
};

struct RecorderOptions {
    bool record_all = false;
    bool regex = false;
    bool quiet = false;
    bool snapshot = false;
    std::vector<std::string> topics;
    std::string prefix = "";
    std::string output_uri = "rosbag2_output";
    int queue_depth = 10;
};

class Recorder : public rclcpp::Node {
public:
    explicit Recorder(const RecorderOptions& options)
        : Node("rosbag2_recorder"),
          options_(options),
          queue_size_(0),
          writing_enabled_(true)
    {
        queue_ = std::make_unique<std::queue<OutgoingMessage>>();
    }

    void subscribe(const std::string& topic) {
        if (currently_recording_.find(topic) != currently_recording_.end()) {
            return;
        }

        // Use SensorDataQoS for best_effort reliability to handle high-frequency
        // sensor data streams (LIDAR, images, etc.) without congestion
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(options_.queue_depth);

        // Create a generic_subscription that is type-agnostic.
        // This uses rmw_serialized_message to receive any message type
        // without knowing the type at compile time.
        auto subscription = this->create_generic_subscription(
            topic,
            "*",  // wildcard type - will be resolved at runtime
            qos.best_effort(),
            [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
                this->doQueue(serialized_msg, topic);
            }
        );

        subscribers_.push_back(subscription);
        currently_recording_.insert(topic);

        if (!options_.quiet) {
            RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
        }
    }

    //! Callback to be invoked to save messages into a queue
    void doQueue(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
                 const std::string& topic) {
        // Use the node's synchronized clock source (this->now()) to timestamp
        // the data for consistent system-wide playback.
        // This ensures timestamps are aligned with the ROS Domain Clock.
        rclcpp::Time receipt_time = this->now();

        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_->push(OutgoingMessage(topic, serialized_msg, receipt_time, ""));
        queue_size_ += serialized_msg->size();
        queue_condition_.notify_all();
    }

    bool isSubscribed(const std::string& topic) const {
        return currently_recording_.find(topic) != currently_recording_.end();
    }

private:
    RecorderOptions options_;
    std::set<std::string> currently_recording_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscribers_;

    std::unique_ptr<std::queue<OutgoingMessage>> queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    uint64_t queue_size_;
    bool writing_enabled_;
};

}  // namespace rosbag2_recorder