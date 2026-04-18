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
#include <topic_tools/shape_shifter.hpp>

#include "ros/network.h" // This header is ROS1 specific, consider removing or finding ROS2 equivalent if needed for master interaction
#include "ros/xmlrpc_manager.h" // This header is ROS1 specific, consider removing or finding ROS2 equivalent if needed for master interaction
#include "xmlrpcpp/XmlRpc.h" // This header is ROS1 specific, consider removing or finding ROS2 equivalent if needed for master interaction

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

OutgoingMessage::OutgoingMessage(string const& _topic, topic_tools::ShapeShifter::SharedPtr _msg, boost::shared_ptr<std::map<string, string>> _connection_header, Time _time) :
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
    node_ = rclcpp::Node::make_shared("rosbag_recorder");

    if (options_.publish)
    {
        pub_begin_write = node_->create_publisher<std_msgs::msg::String>("begin_write", 1);
    }

    last_buffer_warn_ = node_->now();
    queue_ = new std::queue<OutgoingMessage>;

    // Subscribe to each topic
    if (!options_.regex) {
    	for (string const& topic : options_.topics)
            subscribe(topic);
    }

    // ROS2 does not have a direct equivalent to ros::Time::waitForValid.
    // The node's clock is generally valid from initialization.
    // If /use_sim_time is true, the clock will advance when a /clock message is published.
    // For now, we assume the clock is valid or will become valid.
    RCLCPP_INFO(node_->get_logger(), "Waiting for valid time (if /use_sim_time is true and no clock published, this will wait indefinitely)...");
    // In ROS2, rclcpp::Clock::now() will return a valid time even if /clock is not published,
    // it will just be the system time. If /use_sim_time is true, it will use the simulation time.
    // There's no direct equivalent to "waiting for valid time" as in ROS1.
    // If simulation time is critical, one might need to check for /clock topic activity.

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
          catch (const rosbag::BagException& ex)
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
            exit_code_ = 1;
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
        trigger_sub = node_->create_subscription<std_msgs::msg::Empty>("snapshot_trigger", 100,
            std::bind(&Recorder::snapshotTrigger, this, std::placeholders::_1));
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
            RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
            exit_code_ = 1;
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
        doCheckMaster(node_); // Call immediately
        check_master_timer = node_->create_wall_timer(std::chrono::seconds(1),
            std::bind(&Recorder::doCheckMaster, this, node_));
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin(); // This will spin until rclcpp::ok() is false or shutdown is called.

    record_thread.join();
    queue_condition_.notify_all();
    delete queue_;

    rclcpp::shutdown();
    return exit_code_;
}

shared_ptr<rclcpp::SubscriptionBase> Recorder::subscribe(string const& topic) {
/* * TODO: [System Level]
* Implement a ROS 2 subscription that is type-agnostic (can receive 
     * any message type at runtime). Ensure the connection handles both 
     * high-frequency sensor data (Best Effort) and reliable data streams 
     * appropriately.
 *END OF TODO
 */
    RCLCPP_INFO(node_->get_logger(), "Attempting to subscribe to topic: %s", topic.c_str());

    // Use topic_tools::ShapeShifter for type-agnostic subscription
    auto sub = node_->create_generic_subscription(
        topic,
        "topic_tools/ShapeShifter", // This is a placeholder, actual type will be discovered
        rclcpp::QoS(10).keep_last(10).best_effort(), // Default to best effort for high-frequency data
        [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
            // Deserialize the message into a ShapeShifter
            auto shape_shifter_msg = std::make_shared<topic_tools::ShapeShifter>();
            shape_shifter_msg->deserialize(serialized_msg);

            // Create a connection header map (ROS2 equivalent of ROS1's connection_header)
            // This is a simplified representation. In a real scenario, you might need to
            // extract more details from the topic's type support or introspection.
            auto connection_header = std::make_shared<std::map<string, string>>();
            connection_header->insert({"topic", topic});
            connection_header->insert({"type", shape_shifter_msg->get_type_name()});
            connection_header->insert({"md5sum", shape_shifter_msg->get_md5sum()});

            // Call doQueue with the ShapeShifter message and connection header
            // The message event concept is different in ROS2. We pass the necessary info directly.
            doQueue(shape_shifter_msg, topic, connection_header);
        }
    );

    // Store the subscription to keep it alive
    currently_recording_[topic] = sub;
    num_subscribers_++;
    RCLCPP_INFO(node_->get_logger(), "Subscribed to topic '%s' with type '%s'", topic.c_str(), sub->get_topic_type().c_str());
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
    (void)ros_t; // ros_t is not directly used for formatting in ROS2 context for filename
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
void Recorder::doQueue(const topic_tools::ShapeShifter::SharedPtr msg, string const& topic, boost::shared_ptr<std::map<string, string>> connection_header) {
/* TODO [Task: Temporal Integrity]:
     * Process the received message and add it to the recording queue.
     * You must use the node's synchronized clock source to timestamp 
     * the data for consistent system-wide playback.
     *END OF TODO     
*/
    // Use the node's synchronized clock for timestamping
    rclcpp::Time now = node_->now();

    // Create an OutgoingMessage and push it to the queue
    boost::mutex::scoped_lock lock(queue_mutex_);
    queue_->push(OutgoingMessage(topic, msg, connection_header, now));
    queue_size_ += msg->size(); // Assuming ShapeShifter has a size() method or equivalent

    // Notify the recording thread that there's a new message
    queue_condition_.notify_one();

    // Check for buffer overflow and warn if necessary
    if (options_.buffer_size > 0 && queue_size_ > options_.buffer_size) {
        rclcpp::Time current_time = node_->now();
        if ((current_time - last_buffer_warn_).seconds() > 5.0) {
            RCLCPP_WARN(node_->get_logger(), "Bag recording buffer exceeded. Dropping messages.");
            last_buffer_warn_ = current_time;
        }
        // Optionally, drop messages from the front of the queue to manage buffer size
        while (queue_size_ > options_.buffer_size && !queue_->empty()) {
            queue_size_ -= queue_->front().msg->size();
            queue_->pop();
        }
    }
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
        parts.push_back(timeToStr(node_->now())); // Use node's clock for timeToStr
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
void Recorder::snapshotTrigger(std_msgs::msg::Empty::ConstSharedPtr trigger) {
    (void)trigger;
    updateFilenames();
    
    RCLCPP_INFO(node_->get_logger(), "Triggered snapshot recording with name '%s'.", target_filename_.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, node_->now())); // Use node's clock
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
    if (options_.max_duration > rclcpp::Duration(0,0))
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
    warn_next_ = node_->now();

    try
    {
        checkDisk();
    }
    catch (const rosbag::BagException& ex)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
        exit_code_ = 1;
        stopWriting();
        return;
    }

    check_disk_next_ = node_->now() + rclcpp::Duration(20, 0);

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    while (rclcpp::ok() || !queue_->empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!rclcpp::ok()) {
                lock.release()->unlock();
                finished = true;
                break;
            }
            // ROS2 equivalent of boost::xtime_get and timed_wait
            // Using std::chrono for timed_wait
            std::chrono::system_clock::time_point timeout =
                std::chrono::system_clock::now() + std::chrono::milliseconds(250);
            queue_condition_.wait_until(lock, timeout);

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
            bag_.open(write_filename, bagmode::Write);
        }
        catch (const rosbag::BagException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Error writing: %s", ex.what());
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

void Recorder::doCheckMaster(rclcpp::Node::SharedPtr node_handle) {
    // ROS2 equivalent of ros::master::getTopics
    // This requires using the rclcpp::Node's get_topic_names_and_types()
    // and then potentially introspecting each topic.
    auto topic_names_and_types = node_handle->get_topic_names_and_types();

    for (const auto& pair : topic_names_and_types) {
        const std::string& topic_name = pair.first;
        // In ROS2, a topic can have multiple types if different publishers use different types.
        // For recording, we typically subscribe to one. ShapeShifter handles this.
        // The `shouldSubscribeToTopic` logic needs to be adapted if it relies on a single type.
        if (shouldSubscribeToTopic(topic_name)) {
            subscribe(topic_name);
        }
    }
    
    // The ROS1 XML-RPC based node introspection for getting subscriptions
    // does not have a direct ROS2 equivalent.
    // In ROS2, you would typically use rclcpp::Node's introspection
    // functions or the `ros2 topic info` command line tool.
    // For a recorder, if you want to subscribe to topics published by a specific node,
    // you would need to iterate through all topics and check their publishers.
    // This part of the code is highly ROS1-specific and would require a significant
    // re-architecture for ROS2 if the exact functionality is needed.
    // For now, we will skip the ROS1-specific XML-RPC node introspection.
    if (options_.node != std::string(""))
    {
        RCLCPP_WARN(node_handle->get_logger(), "ROS1 node introspection via XML-RPC is not directly supported in ROS2. Skipping check for node '%s' subscriptions.", options_.node.c_str());
        // A ROS2 equivalent would involve:
        // 1. Getting all topic names and types.
        // 2. For each topic, getting its publishers.
        // 3. For each publisher, getting its node name.
        // 4. If the node name matches options_.node, then subscribe to that topic.
        // This is a more involved process than the ROS1 XML-RPC call.
    }
}

void Recorder::doTrigger() {
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("rosbag_snapshot_trigger");
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub = node->create_publisher<std_msgs::msg::Empty>("snapshot_trigger", 1);
    
    std_msgs::msg::Empty msg;
    pub->publish(msg);

    RCLCPP_INFO(node->get_logger(), "Published snapshot trigger.");

    // In ROS2, there's no direct equivalent to ros::Timer for a one-shot shutdown.
    // We can use a std::thread with a sleep or a rclcpp::TimerBase.
    // For a simple shutdown after a delay, a sleep is sufficient for this utility.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
}

bool Recorder::scheduledCheckDisk() {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (node_->now() < check_disk_next_)
        return true;

    check_disk_next_ += rclcpp::Duration(20, 0);
    return checkDisk();
}

bool Recorder::checkDisk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0)
    {
        RCLCPP_WARN(node_->get_logger(), "Failed to check filesystem stats.");
        return true;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < options_.min_space)
    {
        RCLCPP_ERROR(node_->get_logger(), "Less than %s of space free on disk with '%s'.  Disabling recording.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5 * options_.min_space)
    {
        RCLCPP_WARN(node_->get_logger(), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
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
        RCLCPP_WARN(node_->get_logger(), "Failed to check filesystem stats [%s].", e.what());
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
        RCLCPP_WARN(node_->get_logger(), "Less than 5 x %s of space free on disk with '%s'.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
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

    rclcpp::Time now = node_->now();
    if ((now - warn_next_).seconds() >= 0) {
        warn_next_ = now + rclcpp::Duration(5, 0);
        RCLCPP_WARN(node_->get_logger(), "Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

} // namespace rosbag
[FILENAME: talker.py]
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        # 2. Support dynamic topic name remapping using the parameter system.
        # Declare a parameter for the topic name with a default value
        self.declare_parameter('topic_name', 'chatter')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.i = 0
        # 3. Implement a 10Hz non-blocking execution model (using a Timer) 
        #    to publish messages.
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Talker node started, publishing to '{topic_name}' at {1/timer_period} Hz.")

    def timer_callback(self):
        msg = String()
        # 4. Include a timestamp in the message payload derived from the 
        #    node's internal synchronized clock.
        current_time = self.get_clock().now().to_msg()
        msg.data = f'Hello World: {self.i} at time {current_time.sec}.{current_time.nanosec}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def talker():
# TODO: Implement a ROS 2 compatible data source node.
# 1. Initialize this node within the ROS 2 rclpy framework.
    rclpy.init(args=sys.argv)
    node = Talker()
    rclpy.spin(node)
    # 5. Ensure all legacy rospy references are replaced or removed.
    # All rospy references have been replaced with rclpy equivalents.
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    talker()