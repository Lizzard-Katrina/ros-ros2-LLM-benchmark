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

#include "rosbag/recorder.h"

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
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include "ros/network.h"
#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using rclcpp::Time;

namespace rosbag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, rclcpp::SerializedMessage::ConstSharedPtr _msg, boost::shared_ptr<rclcpp::Parameter> _connection_header, Time _time) :
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
    compression(compression::Uncompressed),
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

    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("recorder");

    if (!node->get_node_waitables_interface()->get_rcl_node()->is_valid())
        return 0;

    if (options_.publish)
    {
        pub_begin_write = node->create_publisher<std_msgs::msg::String>("begin_write", 1);
    }

    last_buffer_warn_ = rclcpp::Time();
    queue_ = new std::queue<OutgoingMessage>;

    // Subscribe to each topic
    if (!options_.regex) {
    	for (string const& topic : options_.topics)
            subscribe(topic);
    }

    if (!rclcpp::ok())
      RCLCPP_WARN(node->get_logger(), "/use_sim_time set to true and no clock published.  Still waiting for valid time...");

    rclcpp::Time::waitForValid();

    start_time_ = rclcpp::Time::now();

    // Don't bother doing anything if we never got a valid time
    if (!rclcpp::ok())
        return 0;

    rclcpp::Subscription trigger_sub;

    // Spin up a thread for writing to the file
    std::thread record_thread;
    if (options_.snapshot)
    {
        record_thread = std::thread([this]() {
          try
          {
            this->doRecordSnapshotter();
          }
          catch (const rosbag::BagException& ex)
          {
            RCLCPP_ERROR(node->get_logger(), ex.what());
            exit_code_ = 1;
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR(node->get_logger(), ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            RCLCPP_ERROR(node->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });

        // Subscribe to the snapshot trigger
        trigger_sub = node->create_subscription<std_msgs::msg::Empty>("snapshot_trigger", 100, std::bind(&Recorder::snapshotTrigger, this, std::placeholders::_1));
    }
    else
    {
        record_thread = std::thread([this]() {
          try
          {
            this->doRecord();
          }
          catch (const rosbag::BagException& ex)
          {
            RCLCPP_ERROR(node->get_logger(), ex.what());
            exit_code_ = 1;
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR(node->get_logger(), ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            RCLCPP_ERROR(node->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });
    }

    rclcpp::TimerBase::SharedPtr check_master_timer;
    if (options_.record_all || options_.regex || (options_.node != std::string("")))
    {
        // check for master first
        doCheckMaster(rclcpp::TimerEvent(), node);
        check_master_timer = node->create_wall_timer(std::chrono::seconds(1), std::bind(&Recorder::doCheckMaster, this, std::placeholders::_1, node));
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    record_thread.join();
    queue_condition_.notify_all();
    delete queue_;

    return exit_code_;
}

rclcpp::Subscription::SharedPtr Recorder::subscribe(string const& topic) {
    auto node = rclcpp::Node::make_shared("recorder");
    auto sub = node->create_subscription<rclcpp::SerializedMessage>("topic", 10, std::bind(&Recorder::doQueue, this, std::placeholders::_1, topic));
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
void Recorder::doQueue(const rclcpp::SerializedMessage::SharedPtr msg, string const& topic) {
    OutgoingMessage out(topic, msg, nullptr, rclcpp::Time::now());
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_->push(out);
        queue_size_ += out.msg->size();
    }
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
        parts.push_back(timeToStr(rclcpp::Time::now()));
    if (options_.split)
        parts.push_back(boost::lexical_cast<string>(split_count_));

    if (parts.size() == 0)
    {
      throw BagException("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
    }

    target_filename_ = parts[0];
    for (unsigned int i = 1; i < parts.size(); i++)
        target_filename_ += string("_") + parts[i];

    target_filename_ += string(".bag");
    write_filename_ = target_filename_ + string(".active");
}

//! Callback to be invoked to actually do the recording
void Recorder::snapshotTrigger(const std_msgs::msg::Empty::SharedPtr trigger) {
    (void)trigger;
    updateFilenames();
    
    RCLCPP_INFO(rclcpp::get_logger("recorder"), "Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, rclcpp::Time::now()));
        queue_      = new std::queue<OutgoingMessage>;
        queue_size_ = 0;
    }

    queue_condition_.notify_all();
}

void Recorder::startWriting() {
    bag_.setCompression(options_.compression);
    bag_.setChunkThreshold(options_.chunk_size);

    updateFilenames();
    try {
        bag_.open(write_filename_, bagmode::Write);
    }
    catch (const rosbag::BagException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Error writing: %s", e.what());
        exit_code_ = 1;
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("recorder"), "Recording to '%s'.", target_filename_.c_str());

    if (options_.repeat_latched)
    {
        // Start each new bag file with copies of all latched messages.
        rclcpp::Time now = rclcpp::Time::now();
        for (auto const& out : latched_msgs_)
        {
            // Overwrite the original receipt time, otherwise the new bag will
            // have a gap before the new messages start.
            bag_.write(out.second.topic, now, *out.second.msg, out.second.connection_header);
        }
    }

    if (options_.publish)
    {
        std_msgs::msg::String msg;
        msg.data = target_filename_.c_str();
        pub_begin_write->publish(msg);
    }
}

void Recorder::stopWriting() {
    RCLCPP_INFO(rclcpp::get_logger("recorder"), "Closing '%s'.", target_filename_.c_str());
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
                RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
            }
            current_files_.pop_front();
        }
    }
}

bool Recorder::checkSize()
{
    if (options_.max_size > 0)
    {
        if (bag_.getSize() > options_.max_size)
        {
            if (options_.split)
            {
                stopWriting();
                split_count_++;
                checkNumSplits();
                startWriting();
            } else {
                rclcpp::shutdown();
                return true;
            }
        }
    }
    return false;
}

bool Recorder::checkDuration(const rclcpp::Time& t)
{
    if (options_.max_duration > rclcpp::Duration(0))
    {
        if (t - start_time_ > options_.max_duration)
        {
            if (options_.split)
            {
                while (start_time_ + options_.max_duration < t)
                {
                    stopWriting();
                    split_count_++;
                    checkNumSplits();
                    start_time_ += options_.max_duration;
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
    warn_next_ = rclcpp::Time();

    try
    {
        checkDisk();
    }
    catch (const rosbag::BagException& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("recorder"), ex.what());
        exit_code_ = 1;
        stopWriting();
        return;
    }

    check_disk_next_ = rclcpp::Time::now() + rclcpp::Duration::from_seconds(20.0);

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("recorder");
    while (rclcpp::ok() || !queue_->empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!rclcpp::ok()) {
                lock.unlock();
                finished = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            if (checkDuration(rclcpp::Time::now()))
            {
                finished = true;
                break;
            }
        }
        if (finished)
            break;

        OutgoingMessage out = queue_->front();
        queue_->pop();
        queue_size_ -= out.msg->size();
        
        lock.unlock();
        
        if (checkSize())
            break;

        if (checkDuration(out.time))
            break;

        try
        {
            if (scheduledCheckDisk() && checkLogging())
                bag_.write(out.topic, out.time, *out.msg, out.connection_header);
        }
        catch (const rosbag::BagException& ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("recorder"), ex.what());
            exit_code_ = 1;
            break;
        }
    }

    stopWriting();
}

void Recorder::doRecordSnapshotter() {
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("recorder");
  
    while (rclcpp::ok() || !queue_queue_.empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        while (queue_queue_.empty()) {
            if (!rclcpp::ok())
                return;
            queue_condition_.wait(lock);
        }
        
        OutgoingQueue out_queue = queue_queue_.front();
        queue_queue_.pop();
        
        lock.unlock();
        
        string target_filename = out_queue.filename;
        string write_filename  = target_filename + string(".active");
        
        try {
            bag_.open(write_filename, bagmode::Write);
        }
        catch (const rosbag::BagException& ex) {
            RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Error writing: %s", ex.what());
            return;
        }

        while (!out_queue.queue->empty()) {
            OutgoingMessage out = out_queue.queue->front();
            out_queue.queue->pop();

            bag_.write(out.topic, out.time, *out.msg);
        }

        stopWriting();
    }
}

void Recorder::doCheckMaster(rclcpp::TimerEvent const& e, rclcpp::Node::SharedPtr node_handle) {
    (void)e;
    (void)node_handle;
    std::vector<rclcpp::TopicEndpointInfo> topics;
    if (rclcpp::get_topic_endpoints(topics)) {
	for (const auto& t : topics) {
	    if (shouldSubscribeToTopic(t.topic_name))
	        subscribe(t.topic_name);
	}
    }
    
    if (options_.node != std::string(""))
    {

      XmlRpc::XmlRpcValue req;
      req[0] = rclcpp::get_node_name();
      req[1] = options_.node;
      XmlRpc::XmlRpcValue resp;
      XmlRpc::XmlRpcValue payload;

      if (rclcpp::execute("lookupNode", req, resp, payload, true))
      {
        std::string peer_host;
        uint32_t peer_port;

        if (!rclcpp::split_uri(static_cast<std::string>(resp[2]), peer_host, peer_port))
        {
          RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
        } else {

          XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
          XmlRpc::XmlRpcValue req2;
          XmlRpc::XmlRpcValue resp2;
          req2[0] = rclcpp::get_node_name();
          c.execute("getSubscriptions", req2, resp2);
          
          if (!c.isFault() && resp2.valid() && resp2.size() > 0 && static_cast<int>(resp2[0]) == 1)
          {
            for(int i = 0; i < resp2[2].size(); i++)
            {
              if (shouldSubscribeToTopic(resp2[2][i][0], true))
                subscribe(resp2[2][i][0]);
            }
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
          }
        }
      }
    }
}

void Recorder::doTrigger() {
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("recorder");
    auto pub = node->create_publisher<std_msgs::msg::Empty>("snapshot_trigger", 1);
    pub->publish(std_msgs::msg::Empty());

    rclcpp::TimerBase::SharedPtr terminate_timer = node->create_wall_timer(std::chrono::seconds(1), std::bind(&rclcpp::shutdown));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}

bool Recorder::scheduledCheckDisk() {
    std::lock_guard<std::mutex> lock(check_disk_mutex_);

    if (rclcpp::Time::now() < check_disk_next_)
        return true;

    check_disk_next_ += rclcpp::Duration::from_seconds(20.0);
    return checkDisk();
}

bool Recorder::checkDisk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("recorder"), "Failed to check filesystem stats.");
        return true;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < options_.min_space)
    {
        RCLCPP_ERROR(rclcpp::get_logger("recorder"), "Less than %s of space free on disk with '%s'.  Disabling recording.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5 * options_.min_space)
    {
        RCLCPP_WARN(rclcpp::get_logger("recorder"), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
    }
    else
    {
        writing_enabled_ = true;
    }
#else
    boost::filesystem::path p(boost::filesystem::system_complete(bag_.getFileName().c_str()));
    p = p.parent_path();
    boost::filesystem::space_info info;
    try
    {
        info = boost::filesystem::space(p);
    }
    catch (const boost::filesystem::filesystem_error& e) 
    { 
        RCLCPP_WARN(rclcpp::get_logger("recorder"), "Failed to check filesystem stats [%s].", e.what());
        writing_enabled_ = false;
        return false;
    }
    if ( info.available < options_.min_space)
    {
        writing_enabled_ = false;
        throw BagException("Less than " + options_.min_space_str + " of space free on disk with " + bag_.getFileName() + ". Disabling recording.");
    }
    else if (info.available < 5 * options_.min_space)
    {
        RCLCPP_WARN(rclcpp::get_logger("recorder"), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = true;
    }
    else
    {
        writing_enabled_ = true;
    }
#endif
    return true;
}

bool Recorder::checkLogging() {
    if (writing_enabled_)
        return true;

    rclcpp::Time now = rclcpp::Time::now();
    if (now >= warn_next_) {
        warn_next_ = now + rclcpp::Duration::from_seconds(5.0);
        RCLCPP_WARN(rclcpp::get_logger("recorder"), "Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

} // namespace rosbag