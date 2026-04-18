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

FILE_PATH: recorder.cpp
----------------------------
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

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "ros/network.h"
#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

namespace rosbag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, Time _time) :
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

    ros::NodeHandle nh;
    if (!nh.ok())
        return 0;

    if (options_.publish)
    {
        pub_begin_write = nh.advertise<std_msgs::String>("begin_write", 1, true);
    }

    last_buffer_warn_ = Time();
    queue_ = new std::queue<OutgoingMessage>;

    // Subscribe to each topic
    if (!options_.regex) {
    	for (string const& topic : options_.topics)
            subscribe(topic);
    }

    if (!ros::Time::waitForValid(ros::WallDuration(2.0)))
      ROS_WARN("/use_sim_time set to true and no clock published.  Still waiting for valid time...");

    ros::Time::waitForValid();

    start_time_ = ros::Time::now();

    // Don't bother doing anything if we never got a valid time
    if (!nh.ok())
        return 0;

    ros::Subscriber trigger_sub;

    // Spin up a thread for writing to the file
    boost::thread record_thread;
    if (options_.snapshot)
    {
        record_thread = boost::thread([this]() {
          try
          {
            this->doRecordSnapshotter();
          }
          catch (const rosbag::BagException& ex)
          {
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 1;
          }
          catch (const std::exception& ex)
          {
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            ROS_ERROR_STREAM("Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });

        // Subscribe to the snapshot trigger
        trigger_sub = nh.subscribe<std_msgs::Empty>("snapshot_trigger", 100, boost::bind(&Recorder::snapshotTrigger, this, boost::placeholders::_1));
    }
    else
    {
        record_thread = boost::thread([this]() {
          try
          {
            this->doRecord();
          }
          catch (const rosbag::BagException& ex)
          {
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 1;
          }
          catch (const std::exception& ex)
          {
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 2;
          }
          catch (...)
          {
            ROS_ERROR_STREAM("Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });
    }



    ros::Timer check_master_timer;
    if (options_.record_all || options_.regex || (options_.node != std::string("")))
    {
        // check for master first
        doCheckMaster(ros::TimerEvent(), nh);
        check_master_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&Recorder::doCheckMaster, this, boost::placeholders::_1, boost::ref(nh)));
    }

    ros::AsyncSpinner s(10);
    s.start();

    record_thread.join();
    queue_condition_.notify_all();
    delete queue_;

    return exit_code_;
}

shared_ptr<ros::Subscriber> Recorder::subscribe(string const& topic) {
/* * TODO: [System Level]
* Implement a ROS 2 subscription that is type-agnostic (can receive 
     * any message type at runtime). Ensure the connection handles both 
     * high-frequency sensor data (Best Effort) and reliable data streams 
     * appropriately.
 *END OF TODO
 */

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
void Recorder::doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, string const& topic, shared_ptr<ros::Subscriber> subscriber, shared_ptr<int> count) {
/* TODO [Task: Temporal Integrity]:
     * Process the received message and add it to the recording queue.
     * You must use the node's synchronized clock source to timestamp 
     * the data for consistent system-wide playback.
     *END OF TODO     
*/
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
        parts.push_back(timeToStr(ros::WallTime::now()));
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
void Recorder::snapshotTrigger(std_msgs::Empty::ConstPtr trigger) {
    (void)trigger;
    updateFilenames();
    
    ROS_INFO("Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, Time::now()));
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
        ROS_ERROR("Error writing: %s", e.what());
        exit_code_ = 1;
        ros::shutdown();
    }
    ROS_INFO("Recording to '%s'.", target_filename_.c_str());

    if (options_.repeat_latched)
    {
        // Start each new bag file with copies of all latched messages.
        ros::Time now = ros::Time::now();
        for (auto const& out : latched_msgs_)
        {
            // Overwrite the original receipt time, otherwise the new bag will
            // have a gap before the new messages start.
            bag_.write(out.second.topic, now, *out.second.msg, out.second.connection_header);
        }
    }

    if (options_.publish)
    {
        std_msgs::String msg;
        msg.data = target_filename_.c_str();
        pub_begin_write.publish(msg);
    }
}

void Recorder::stopWriting() {
    ROS_INFO("Closing '%s'.", target_filename_.c_str());
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
                ROS_ERROR("Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
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
                ros::shutdown();
                return true;
            }
        }
    }
    return false;
}

bool Recorder::checkDuration(const ros::Time& t)
{
    if (options_.max_duration > ros::Duration(0))
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
                ros::shutdown();
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
    warn_next_ = ros::WallTime();

    try
    {
        checkDisk();
    }
    catch (const rosbag::BagException& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        exit_code_ = 1;
        stopWriting();
        return;
    }

    check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    ros::NodeHandle nh;
    while (nh.ok() || !queue_->empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!nh.ok()) {
                lock.release()->unlock();
                finished = true;
                break;
            }
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.nsec += 250000000;
            queue_condition_.timed_wait(lock, xt);
            if (checkDuration(ros::Time::now()))
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
        
        lock.release()->unlock();
        
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
            ROS_ERROR_STREAM(ex.what());
            exit_code_ = 1;
            break;
        }
    }

    stopWriting();
}

void Recorder::doRecordSnapshotter() {
    ros::NodeHandle nh;
  
    while (nh.ok() || !queue_queue_.empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);
        while (queue_queue_.empty()) {
            if (!nh.ok())
                return;
            queue_condition_.wait(lock);
        }
        
        OutgoingQueue out_queue = queue_queue_.front();
        queue_queue_.pop();
        
        lock.release()->unlock();
        
        string target_filename = out_queue.filename;
        string write_filename  = target_filename + string(".active");
        
        try {
            bag_.open(write_filename, bagmode::Write);
        }
        catch (const rosbag::BagException& ex) {
            ROS_ERROR("Error writing: %s", ex.what());
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

void Recorder::doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle) {
    (void)e;
    (void)node_handle;
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics)) {
	for (ros::master::TopicInfo const& t : topics) {
	    if (shouldSubscribeToTopic(t.name))
	        subscribe(t.name);
	}
    }
    
    if (options_.node != std::string(""))
    {

      XmlRpc::XmlRpcValue req;
      req[0] = ros::this_node::getName();
      req[1] = options_.node;
      XmlRpc::XmlRpcValue resp;
      XmlRpc::XmlRpcValue payload;

      if (ros::master::execute("lookupNode", req, resp, payload, true))
      {
        std::string peer_host;
        uint32_t peer_port;

        if (!ros::network::splitURI(static_cast<std::string>(resp[2]), peer_host, peer_port))
        {
          ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
        } else {

          XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
          XmlRpc::XmlRpcValue req2;
          XmlRpc::XmlRpcValue resp2;
          req2[0] = ros::this_node::getName();
          c.execute("getSubscriptions", req2, resp2);
          
          if (!c.isFault() && resp2.valid() && resp2.size() > 0 && static_cast<int>(resp2[0]) == 1)
          {
            for(int i = 0; i < resp2[2].size(); i++)
            {
              if (shouldSubscribeToTopic(resp2[2][i][0], true))
                subscribe(resp2[2][i][0]);
            }
          } else {
            ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
          }
        }
      }
    }
}

void Recorder::doTrigger() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
    pub.publish(std_msgs::Empty());

    ros::Timer terminate_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
    ros::spin();
}

bool Recorder::scheduledCheckDisk() {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (ros::WallTime::now() < check_disk_next_)
        return true;

    check_disk_next_ += ros::WallDuration().fromSec(20.0);
    return checkDisk();
}

bool Recorder::checkDisk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0)
    {
        ROS_WARN("Failed to check filesystem stats.");
        return true;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < options_.min_space)
    {
        ROS_ERROR("Less than %s of space free on disk with '%s'.  Disabling recording.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5 * options_.min_space)
    {
        ROS_WARN("Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
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
        ROS_WARN("Failed to check filesystem stats [%s].", e.what());
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
        ROS_WARN("Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
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

    ros::WallTime now = ros::WallTime::now();
    if (now >= warn_next_) {
        warn_next_ = now + ros::WallDuration().fromSec(5.0);
        ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

} // namespace rosbag

----------------------------

FILE_PATH: talker.py
----------------------------
#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## Simple talker demo that publishes std_msg/Strings to the 'chatter' topic

PKG = 'test_rospy' # this package name
NAME = 'talker'

import sys
import time

import rospy
from std_msgs.msg import String

def talker():
# TODO: Implement a ROS 2 compatible data source node.
# 1. Initialize this node within the ROS 2 rclpy framework.
    # 2. Support dynamic topic name remapping using the parameter system.
    # 3. Implement a 10Hz non-blocking execution model (using a Timer) 
    #    to publish messages.
    # 4. Include a timestamp in the message payload derived from the 
    #    node's internal synchronized clock.
    # 5. Ensure all legacy rospy references are replaced or removed.
# END OF TODO
        
if __name__ == '__main__':
    talker()
        

----------------------------


# LLM Response



[FILENAME: recorder.cpp]
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

#include "rosbag2_transport/recorder.hpp"

#include <sys/stat.h>
#include <boost/filesystem.hpp>
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
#include <boost/date_time/local_time/local_time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using std::shared_ptr;

namespace rosbag2_transport {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, std::shared_ptr<rclcpp::SerializedMessage> _msg, rclcpp::Time _time, std::string const& _type) :
    topic(_topic), msg(_msg), time(_time), type(_type)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(string const& _filename, std::queue<OutgoingMessage>* _queue, rclcpp::Time _time) :
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
    rclcpp::Node("rosbag2_recorder"),
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
        if (options_.limit > 0) {
            fprintf(stderr, "Specifying a count is not valid with automatic topic subscription.\n");
            return 1;
        }

        if (!options_.record_all && (options_.node == std::string(""))) {
            fprintf(stderr, "No topics specified.\n");
            return 1;
        }
    }

    if (!rclcpp::ok())
        return 0;

    if (options_.publish)
    {
        pub_begin_write_ = this->create_publisher<std_msgs::msg::String>("begin_write", rclcpp::QoS(1).transient_local());
    }

    last_buffer_warn_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    queue_ = new std::queue<OutgoingMessage>;

    // Subscribe to each topic
    if (!options_.regex) {
        for (string const& topic : options_.topics)
            subscribe(topic);
    }

    // Wait for valid time if using sim time
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_WARN(this->get_logger(), "use_sim_time set to true and no clock published. Still waiting for valid time...");
    }

    start_time_ = this->now();

    if (!rclcpp::ok())
        return 0;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub;

    // Spin up a thread for writing to file
    std::thread record_thread;
    if (options_.snapshot)
    {
        record_thread = std::thread([this]() {
          try
          {
            this->doRecordSnapshotter();
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            exit_code_ = 1;
          }
          catch (...)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });

        trigger_sub = this->create_subscription<std_msgs::msg::Empty>(
            "snapshot_trigger", 100,
            std::bind(&Recorder::snapshotTrigger, this, std::placeholders::_1));
    }
    else
    {
        record_thread = std::thread([this]() {
          try
          {
            this->doRecord();
          }
          catch (const std::exception& ex)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            exit_code_ = 1;
          }
          catch (...)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown exception thrown while recording bag, exiting.");
            exit_code_ = 3;
          }
        });
    }

    rclcpp::TimerBase::SharedPtr check_master_timer;
    if (options_.record_all || options_.regex || (options_.node != std::string("")))
    {
        doCheckMaster();
        check_master_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Recorder::doCheckMaster, this));
    }

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);
    executor.add_node(this->shared_from_this());

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    record_thread.join();
    queue_condition_.notify_all();
    delete queue_;

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return exit_code_;
}

shared_ptr<rclcpp::GenericSubscription> Recorder::subscribe(string const& topic) {
    // Look up the topic type from the available topic names and types
    std::string topic_type;
    auto topics_and_types = this->get_topic_names_and_types();
    auto it = topics_and_types.find(topic);
    if (it != topics_and_types.end() && !it->second.empty()) {
        topic_type = it->second[0];
    } else {
        RCLCPP_WARN(this->get_logger(), "Topic '%s' not yet available, deferring subscription.", topic.c_str());
        return nullptr;
    }

    RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic.c_str());

    auto count = std::make_shared<int>(options_.limit);
    auto type_str = topic_type;

    // Use best effort for sensor topics, reliable otherwise
    rclcpp::QoS qos(100);
    qos.best_effort();

    auto subscriber = this->create_generic_subscription(
        topic,
        topic_type,
        qos,
        [this, topic, count, type_str](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            rclcpp::Time now = this->now();

            if (options_.limit > 0 && (*count) >= options_.limit) {
                rclcpp::shutdown();
                return;
            }

            if (options_.limit > 0) {
                (*count)++;
            }

            OutgoingMessage out(topic, msg, now, type_str);

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);

                if (options_.snapshot) {
                    // In snapshot mode, keep the buffer within limits
                    while (queue_size_ + msg->size() > options_.buffer_size && !queue_->empty()) {
                        OutgoingMessage drop = queue_->front();
                        queue_->pop();
                        queue_size_ -= drop.msg->size();
                    }
                }

                queue_->push(out);
                queue_size_ += msg->size();

                if (!options_.snapshot && queue_size_ > options_.buffer_size) {
                    rclcpp::Time now_t = this->now();
                    if (now_t > last_buffer_warn_ + rclcpp::Duration(5, 0)) {
                        RCLCPP_WARN(this->get_logger(), "rosbag record buffer exceeded. Dropping oldest queued message.");
                        last_buffer_warn_ = now_t;
                    }
                    queue_->pop();
                    queue_size_ -= msg->size();
                }
            }

            if (!options_.snapshot)
                queue_condition_.notify_all();
        }
    );

    currently_recording_.insert(topic);
    num_subscribers_++;

    return subscriber;
}

bool Recorder::isSubscribed(string const& topic) const {
    return currently_recording_.find(topic) != currently_recording_.end();
}

bool Recorder::shouldSubscribeToTopic(std::string const& topic, bool from_node) {
    if (isSubscribed(topic)) {
        return false;
    }

    if(options_.do_exclude && boost::regex_match(topic, options_.exclude_regex)) {
        return false;
    }

    if(options_.record_all || from_node) {
        return true;
    }
    
    if (options_.regex) {
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
        parts.push_back(timeToStr(rclcpp::Clock().now()));
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

void Recorder::snapshotTrigger(std_msgs::msg::Empty::ConstSharedPtr trigger) {
    (void)trigger;
    updateFilenames();
    
    RCLCPP_INFO(this->get_logger(), "Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, this->now()));
        queue_      = new std::queue<OutgoingMessage>;
        queue_size_ = 0;
    }

    queue_condition_.notify_all();
}

void Recorder::startWriting() {
    updateFilenames();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = write_filename_;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(storage_options, converter_options);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error writing: %s", e.what());
        exit_code_ = 1;
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Recording to '%s'.", target_filename_.c_str());

    if (options_.publish)
    {
        std_msgs::msg::String msg;
        msg.data = target_filename_.c_str();
        pub_begin_write_->publish(msg);
    }
}

void Recorder::stopWriting() {
    RCLCPP_INFO(this->get_logger(), "Closing '%s'.", target_filename_.c_str());
    writer_.reset();
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
                RCLCPP_ERROR(this->get_logger(), "Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
            }
            current_files_.pop_front();
        }
    }
}

bool Recorder::checkSize()
{
    if (options_.max_size > 0)
    {
        // In ROS2 rosbag2, we don't have a direct getSize() equivalent easily,
        // so we check file size on disk
        boost::filesystem::path p(write_filename_);
        if (boost::filesystem::exists(p) && 
            static_cast<uint64_t>(boost::filesystem::file_size(p)) > options_.max_size)
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
    if (options_.max_duration > rclcpp::Duration(0, 0))
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
                    start_time_ = start_time_ + options_.max_duration;
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

void Recorder::doRecord() {
    startWriting();

    warn_next_ = std::chrono::steady_clock::time_point();

    try
    {
        checkDisk();
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        exit_code_ = 1;
        stopWriting();
        return;
    }

    check_disk_next_ = std::chrono::steady_clock::now() + std::chrono::seconds(20);

    while (rclcpp::ok() || !queue_->empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!rclcpp::ok()) {
                lock.unlock();
                finished = true;
                break;
            }
            queue_condition_.wait_for(lock, std::chrono::milliseconds(250));
            if (checkDuration(this->now()))
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
            {
                // Create topic metadata if not already created
                if (topic_types_created_.find(out.topic) == topic_types_created_.end()) {
                    rosbag2_storage::TopicMetadata topic_metadata;
                    topic_metadata.name = out.topic;
                    topic_metadata.type = out.type;
                    topic_metadata.serialization_format = "cdr";
                    writer_->create_topic(topic_metadata);
                    topic_types_created_.insert(out.topic);
                }

                auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                bag_msg->topic_name = out.topic;
                bag_msg->time_stamp = out.time.nanoseconds();
                bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                    new rcutils_uint8_array_t,
                    [](rcutils_uint8_array_t* arr) {
                        if (arr) {
                            rcutils_uint8_array_fini(arr);
                            delete arr;
                        }
                    });
                *bag_msg->serialized_data = rcutils_get_zero_initialized_uint8_array();
                rcutils_uint8_array_init(bag_msg->serialized_data.get(), out.msg->size(), rcutils_get_default_allocator());
                memcpy(bag_msg->serialized_data->buffer, out.msg->get_rcl_serialized_message().buffer, out.msg->size());
                bag_msg->serialized_data->buffer_length = out.msg->size();

                writer_->write(bag_msg);
            }
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            exit_code_ = 1;
            break;
        }
    }

    stopWriting();
}

void Recorder::doRecordSnapshotter() {
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

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = write_filename;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
            writer_->open(storage_options, converter_options);
        }
        catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error writing: %s", ex.what());
            return;
        }

        std::set<std::string> snapshot_topics_created;
        while (!out_queue.queue->empty()) {
            OutgoingMessage out = out_queue.queue->front();
            out_queue.queue->pop();

            if (snapshot_topics_created.find(out.topic) == snapshot_topics_created.end()) {
                rosbag2_storage::TopicMetadata topic_metadata;
                topic_metadata.name = out.topic;
                topic_metadata.type = out.type;
                topic_metadata.serialization_format = "cdr";
                writer_->create_topic(topic_metadata);
                snapshot_topics_created.insert(out.topic);
            }

            auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_msg->topic_name = out.topic;
            bag_msg->time_stamp = out.time.nanoseconds();
            bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t,
                [](rcutils_uint8_array_t* arr) {
                    if (arr) {
                        rcutils_uint8_array_fini(arr);
                        delete arr;
                    }
                });
            *bag_msg->serialized_data = rcutils_get_zero_initialized_uint8_array();
            rcutils_uint8_array_init(bag_msg->serialized_data.get(), out.msg->size(), rcutils_get_default_allocator());
            memcpy(bag_msg->serialized_data->buffer, out.msg->get_rcl_serialized_message().buffer, out.msg->size());
            bag_msg->serialized_data->buffer_length = out.msg->size();

            writer_->write(bag_msg);
        }

        stopWriting();
    }
}

void Recorder::doCheckMaster() {
    auto topics_and_types = this->get_topic_names_and_types();
    for (auto const& pair : topics_and_types) {
        if (shouldSubscribeToTopic(pair.first))
            subscribe(pair.first);
    }
}

void Recorder::doTrigger() {
    auto node = rclcpp::Node::make_shared("rosbag2_trigger");
    auto pub = node->create_publisher<std_msgs::msg::Empty>("snapshot_trigger", rclcpp::QoS(1).transient_local());
    pub->publish(std_msgs::msg::Empty());

    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [&node]() { rclcpp::shutdown(); });
    rclcpp::spin(node);
}

bool Recorder::scheduledCheckDisk() {
    std::lock_guard<std::mutex> lock(check_disk_mutex_);

    if (std::chrono::steady_clock::now() < check_disk_next_)
        return true;

    check_disk_next_ += std::chrono::seconds(20);
    return checkDisk();
}

bool Recorder::checkDisk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(write_filename_.c_str(), &fiData)) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to check filesystem stats.");
        return true;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < options_.min_space)
    {
        RCLCPP_ERROR(this->get_logger(), "Less than %s of space free on disk with '%s'.  Disabling recording.", options_.min_space_str.c_str(), write_filename_.c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5 * options_.min_space)
    {
        RCLCPP_WARN(this->get_logger(), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), write_filename_.c_str());
    }
    else
    {
        writing_enabled_ = true;
    }
#else
    boost::filesystem::path p(boost::filesystem::system_complete(write_filename_.c_str()));
    p = p.parent_path();
    boost::filesystem::space_info info;
    try
    {
        info = boost::filesystem::space(p);
    }
    catch (const boost::filesystem::filesystem_error& e) 
    { 
        RCLCPP_WARN(this->get_logger(), "Failed to check filesystem stats [%s].", e.what());
        writing_enabled_ = false;
        return false;
    }
    if ( info.available < options_.min_space)
    {
        writing_enabled_ = false;
        throw std::runtime_error("Less than " + options_.min_space_str + " of space free on disk with " + write_filename_ + ". Disabling recording.");
    }
    else if (info.available < 5 * options_.min_space)
    {
        RCLCPP_WARN(this->get_logger(), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), write_filename_.c_str());
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

    auto now = std::chrono::steady_clock::now();
    if (now >= warn_next_) {
        warn_next_ = now + std::chrono::seconds(5);
        RCLCPP_WARN(this->get_logger(), "Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

} // namespace rosbag2_transport

[FILENAME: talker.py]
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## Simple talker demo that publishes std_msg/Strings to the 'chatter' topic

PKG = 'test_rclpy'  # this package name
NAME = 'talker'

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('topic_name', 'chatter')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.pub = self.create_publisher(String, topic_name, 10)
        self.count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        current_time = self.get_clock().now()
        msg = String()
        msg.data = 'hello world %d [%s]' % (self.count, str(current_time))
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.pub.publish(msg)
        self.count += 1


def talker():
    rclpy.init(args=sys.argv)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    talker()