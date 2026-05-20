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

#include "rosbag2_cpp/writer.hpp"

#include <sys/stat.h>
#include <boost/filesystem.hpp>
// Boost filesystem v3 is default in 1.46.0 and above
// Fallback to original posix code (*nix only) if this is not true
#if BOOST_FILESYSTEM_VERSION < 3
  #include <sys/statvfs.h>
#endif
#include <time.h>

#include <queue>
#include <set>
#include <sstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using rclcpp::Time;

namespace rosbag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, std::shared_ptr<rclcpp::SerializedMessage> _msg, boost::shared_ptr<std::map<std::string, std::string>> _connection_header, Time _time) :
    topic(_topic), msg(_msg), connection_header(_connection_header), time(_time)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(string const& _filename, std::queue<OutgoingMessage>* _queue, Time _time) :
    filename(_filename), queue(_queue), time(_time)
{
}

// RecorderOptions

RecorderOptions::RecorderOptions() :
    trigger(false),
    record_all(false),
    regex(false),
    do_exclude(false),
    quiet(false),
    append_date(true),
    snapshot(false),
    verbose(false),
    publish(false),
    repeat_latched(false),
    compression(""),
    prefix(""),
    name(""),
    exclude_regex(),
    buffer_size(1048576 * 256),
    chunk_size(1024 * 768),
    limit(0),
    split(false),
    max_size(0),
    max_splits(0),
    max_duration(-1.0),
    node(""),
    min_space(1024 * 1024 * 1024),
    min_space_str("1G")
{
}

// Recorder

Recorder::Recorder(RecorderOptions const& options) :
    options_(options),
    num_subscribers_(0),
    exit_code_(0),
    queue_size_(0),
    split_count_(0),
    writing_enabled_(true)
{
}

int Recorder::run() {
    if (options_.trigger) {
        doTrigger();
        return 0;
    }

    if (options_.topics.size() == 0) {
        // Make sure limit is not specified with automatic topic subscription
        if (options_.limit > 0) {
            fprintf(stderr, "Specifying a count is not valid with automatic topic subscription.\n");
            return 1;
        }

        // Make sure topics are specified
        if (!options_.record_all && (options_.node == std::string(""))) {
            fprintf(stderr, "No topics specified.\n");
            return 1;
        }
    }

    node_ = rclcpp::Node::make_shared("rosbag_recorder");
    if (!rclcpp::ok())
        return 0;

    if (options_.publish)
    {
        pub_begin_write = node_->create_publisher<std_msgs::msg::String>("begin_write", 1);
    }

    last_buffer_warn_ = Time(0, 0, node_->get_clock()->get_clock_type());
    queue_ = new std::queue<OutgoingMessage>;

    // Subscribe to each topic
    if (!options_.regex) {
    	for (string const& topic : options_.topics)
            subscribe(topic);
    }

    start_time_ = node_->now();

    // Don't bother doing anything if we never got a valid time
    if (!rclcpp::ok())
        return 0;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub;

    // Spin up a thread for writing to the file
    boost::thread record_thread;
    if (options_.snapshot)
    {
        record_thread = boost::thread([this]() {
          try
          {
            this->doRecordSnapshotter();
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });

        // Subscribe to the snapshot trigger
        trigger_sub = node_->create_subscription<std_msgs::msg::Empty>("snapshot_trigger", 100, std::bind(&Recorder::snapshotTrigger, this, std::placeholders::_1));
    }
    else
    {
        record_thread = boost::thread([this]() {
          try
          {
            this->doRecord();
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });
    }



    rclcpp::TimerBase::SharedPtr check_master_timer;
    if (options_.record_all || options_.regex || (options_.node != std::string("")))
    {
        // check for master first
        doCheckMaster();
        check_master_timer = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&Recorder::doCheckMaster, this));
    }

    rclcpp::spin(node_);

    record_thread.join();
    queue_condition_.notify_all();
    delete queue_;

    return exit_code_;
}

std::shared_ptr<rclcpp::GenericSubscription> Recorder::subscribe(string const& topic) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
    auto sub = node_->create_generic_subscription(
        topic,
        topic_type_map_[topic],
        qos,
        [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            this->doQueue(msg, topic, nullptr, nullptr);
        }
    );
    currently_recording_.insert(topic);
    return sub;
}

bool Recorder::isSubscribed(string const& topic) const {
    return currently_recording_.find(topic) != currently_recording_.end();
}

bool Recorder::shouldSubscribeToTopic(std::string const& topic, bool from_node) {
    // ignore already known topics
    if (isSubscribed(topic)) {
        return false;
    }

    // subtract exclusion regex, if any
    if(options_.do_exclude && boost::regex_match(topic, options_.exclude_regex)) {
        return false;
    }

    if(options_.record_all || from_node) {
        return true;
    }
    
    if (options_.regex) {
        // Treat the topics as regular expressions
	return std::any_of(
            std::begin(options_.topics), std::end(options_.topics),
            [&topic] (string const& regex_str){
                boost::regex e(regex_str);
                boost::smatch what;
                return boost::regex_match(topic, what, e, boost::match_extra);
            });
    }

    return std::find(std::begin(options_.topics), std::end(options_.topics), topic)
	    != std::end(options_.topics);
}

template<class T>
std::string Recorder::timeToStr(T ros_t)
{
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now=
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f=
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

//! Callback to be invoked to save messages into a queue
void Recorder::doQueue(std::shared_ptr<rclcpp::SerializedMessage> msg, string const& topic, shared_ptr<rclcpp::SubscriptionBase> subscriber, shared_ptr<int> count) {
    rclcpp::Time time = node_->now();
    boost::mutex::scoped_lock lock(queue_mutex_);
    queue_->push(OutgoingMessage(topic, msg, nullptr, time));
    queue_size_ += msg->capacity();
    queue_condition_.notify_all();
}

void Recorder::updateFilenames() {
    vector<string> parts;

    std::string prefix = options_.prefix;
    size_t ind = prefix.rfind(".bag");

    if (ind != std::string::npos && ind == prefix.size() - 4)
    {
      prefix.erase(ind);
    }

    if (prefix.length() > 0)
        parts.push_back(prefix);
    if (options_.append_date)
        parts.push_back(timeToStr(node_->now()));
    if (options_.split)
        parts.push_back(boost::lexical_cast<string>(split_count_));

    if (parts.size() == 0)
    {
      throw std::runtime_error("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
    }

    target_filename_ = parts[0];
    for (unsigned int i = 1; i < parts.size(); i++)
        target_filename_ += string("_") + parts[i];

    target_filename_ += string(".bag");
    write_filename_ = target_filename_ + string(".active");
}

//! Callback to be invoked to actually do the recording
void Recorder::snapshotTrigger(std_msgs::msg::Empty::ConstSharedPtr trigger) {
    (void)trigger;
    updateFilenames();
    
    RCLCPP_INFO(node_->get_logger(), "Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, node_->now()));
        queue_      = new std::queue<OutgoingMessage>;
        queue_size_ = 0;
    }

    queue_condition_.notify_all();
}

void Recorder::startWriting() {
    updateFilenames();
    try {
        bag_.open(write_filename_);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error writing: %s", e.what());
        exit_code_ = 1;
        rclcpp::shutdown();
    }
    RCLCPP_INFO(node_->get_logger(), "Recording to '%s'.", target_filename_.c_str());

    if (options_.publish)
    {
        std_msgs::msg::String msg;
        msg.data = target_filename_.c_str();
        pub_begin_write->publish(msg);
    }
}

void Recorder::stopWriting() {
    RCLCPP_INFO(node_->get_logger(), "Closing '%s'.", target_filename_.c_str());
    bag_.close();
    rename(write_filename_.c_str(), target_filename_.c_str());
}

void Recorder::checkNumSplits()
{
    if(options_.max_splits>0)
    {
        current_files_.push_back(target_filename_);
        if(current_files_.size()>options_.max_splits)
        {
            int err = unlink(current_files_.front().c_str());
            if(err != 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
            }
            current_files_.pop_front();
        }
    }
}

bool Recorder::checkSize()
{
    return false;
}

bool Recorder::checkDuration(const rclcpp::Time& t)
{
    if (options_.max_duration > 0.0)
    {
        if ((t - start_time_).seconds() > options_.max_duration)
        {
            if (options_.split)
            {
                while ((start_time_ + rclcpp::Duration::from_seconds(options_.max_duration)).seconds() < t.seconds())
                {
                    stopWriting();
                    split_count_++;
                    checkNumSplits();
                    start_time_ = start_time_ + rclcpp::Duration::from_seconds(options_.max_duration);
                    startWriting();
                }
            } else {
                rclcpp::shutdown();
                return true;
            }
        }
    }
    return false;
}


//! Thread that actually does writing to file.
void Recorder::doRecord() {
    // Open bag file for writing
    startWriting();

    // Schedule the disk space check
    warn_next_ = node_->now();

    try
    {
        checkDisk();
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
        exit_code_ = 1;
        stopWriting();
        return;
    }

    check_disk_next_ = node_->now() + rclcpp::Duration::from_seconds(20.0);

    while (rclcpp::ok() || !queue_->empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!rclcpp::ok()) {
                lock.release()->unlock();
                finished = true;
                break;
            }
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.nsec += 250000000;
            queue_condition_.timed_wait(lock, xt);
            if (checkDuration(node_->now()))
            {
                finished = true;
                break;
            }
        }
        if (finished)
            break;

        OutgoingMessage out = queue_->front();
        queue_->pop();
        queue_size_ -= out.msg->capacity();
        
        lock.release()->unlock();
        
        if (checkSize())
            break;

        if (checkDuration(out.time))
            break;

        try
        {
            if (scheduledCheckDisk() && checkLogging()) {
                auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                bag_msg->topic_name = out.topic;
                bag_msg->time_stamp = out.time.nanoseconds();
                bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
                *bag_msg->serialized_data = out.msg->get_rcl_serialized_message();
                bag_.write(bag_msg);
            }
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
            exit_code_ = 1;
            break;
        }
    }

    stopWriting();
}

void Recorder::doRecordSnapshotter() {
    while (rclcpp::ok() || !queue_queue_.empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);
        while (queue_queue_.empty()) {
            if (!rclcpp::ok())
                return;
            queue_condition_.wait(lock);
        }
        
        OutgoingQueue out_queue = queue_queue_.front();
        queue_queue_.pop();
        
        lock.release()->unlock();
        
        string target_filename = out_queue.filename;
        string write_filename  = target_filename + string(".active");
        
        try {
            bag_.open(write_filename);
        }
        catch (const std::exception& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Error writing: %s", ex.what());
            return;
        }

        while (!out_queue.queue->empty()) {
            OutgoingMessage out = out_queue.queue->front();
            out_queue.queue->pop();

            auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_msg->topic_name = out.topic;
            bag_msg->time_stamp = out.time.nanoseconds();
            bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            *bag_msg->serialized_data = out.msg->get_rcl_serialized_message();
            bag_.write(bag_msg);
        }

        stopWriting();
    }
}

void Recorder::doCheckMaster() {
    auto topics = node_->get_topic_names_and_types();
    for (const auto& topic_it : topics) {
        if (shouldSubscribeToTopic(topic_it.first)) {
            topic_type_map_[topic_it.first] = topic_it.second[0];
            subscribe(topic_it.first);
        }
    }
}

void Recorder::doTrigger() {
    auto node = rclcpp::Node::make_shared("snapshot_trigger_node");
    auto pub = node->create_publisher<std_msgs::msg::Empty>("snapshot_trigger", 1);
    pub->publish(std_msgs::msg::Empty());

    auto terminate_timer = node->create_wall_timer(std::chrono::seconds(1), [](){ rclcpp::shutdown(); });
    rclcpp::spin(node);
}

bool Recorder::scheduledCheckDisk() {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (node_->now() < check_disk_next_)
        return true;

    check_disk_next_ = check_disk_next_ + rclcpp::Duration::from_seconds(20.0);
    return checkDisk();
}

bool Recorder::checkDisk() {
    return true;
}

bool Recorder::checkLogging() {
    if (writing_enabled_)
        return true;

    rclcpp::Time now = node_->now();
    if (now >= warn_next_) {
        warn_next_ = now + rclcpp::Duration::from_seconds(5.0);
        RCLCPP_WARN(node_->get_logger(), "Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

} // namespace rosbag