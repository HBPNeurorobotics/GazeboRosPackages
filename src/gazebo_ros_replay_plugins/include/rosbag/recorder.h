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

#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <sys/stat.h>
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <time.h>

#include <queue>
#include <string>
#include <vector>
#include <list>
#include <atomic>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>
#include <boost/unordered_map.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/bag.h"
#include "rosbag/stream.h"
#include "rosbag/macros.h"

namespace rosbag {

class ROSBAG_DECL OutgoingMessage
{
public:
    OutgoingMessage(std::string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time);

    std::string                         topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    boost::shared_ptr<ros::M_string>    connection_header;
    ros::Time                           time;
};

class ROSBAG_DECL OutgoingQueue
{
public:
    OutgoingQueue(std::string const& _filename, std::queue<OutgoingMessage>* _queue, ros::Time _time);

    std::string                  filename;
    std::queue<OutgoingMessage>* queue;
    ros::Time                    time;
};

struct ROSBAG_DECL RecorderOptions
{
    RecorderOptions();

    bool            trigger;
    bool            record_all;
    bool            regex;
    bool            do_exclude;
    bool            quiet;
    bool            append_date;
    bool            snapshot;
    bool            verbose;
    CompressionType compression;
    std::string     prefix;
    std::string     name;
    std::string     path;
    boost::regex    exclude_regex;
    uint32_t        buffer_size;
    uint32_t        chunk_size;
    uint32_t        limit;
    bool            split;
    uint64_t        max_size;
    uint32_t        max_splits;
    ros::Duration   max_duration;
    std::string     node;
    unsigned long long min_space;
    std::string min_space_str;

    std::vector<std::string> topics;

    // NRP: allowable list of types published by gazebo, others are ignored
    std::vector<std::string> gazebo_type_whitelist;

    // NRP: rate limit for recording gazebo generated topics (in seconds)
    ros::Duration gazebo_rate_limit;
};

class ROSBAG_DECL Recorder
{
public:
    Recorder(RecorderOptions const& options);

    void doTrigger();

    bool isSubscribed(std::string const& topic) const;

    boost::shared_ptr<ros::Subscriber> subscribe(std::string const& topic);

    int run();

    inline void stop() { running_ = false; };

private:
    void printUsage();

    void updateFilenames();
    void startWriting();
    void stopWriting();

    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();

    void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    //    void doQueue(topic_tools::ShapeShifter::ConstPtr msg, std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
    void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
    void doRecord();
    void checkNumSplits();
    bool checkSize();
    bool checkDuration(const ros::Time&);
    void doRecordSnapshotter();
    void doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);

    bool shouldSubscribeToTopic(std::string const& topic, bool from_node = false);

    template<class T>
    static std::string timeToStr(T ros_t);

private:
    RecorderOptions               options_;

    Bag                           bag_;

    std::string                   target_filename_;
    std::string                   write_filename_;
    std::list<std::string>        current_files_;

    std::set<std::string>         currently_recording_;
    int                           num_subscribers_;

    int                           exit_code_;

    boost::condition_variable_any queue_condition_;
    boost::mutex                  queue_mutex_;
    std::queue<OutgoingMessage>*  queue_;
    uint64_t                      queue_size_;
    uint64_t                      max_queue_size_;

    uint64_t                      split_count_;

    std::queue<OutgoingQueue>     queue_queue_;

    ros::Time                     last_buffer_warn_;

    ros::Time                     start_time_;

    bool                          writing_enabled_;
    boost::mutex                  check_disk_mutex_;
    ros::WallTime                 check_disk_next_;
    ros::WallTime                 warn_next_;

    std::atomic<bool>                         running_;
		std::vector<boost::shared_ptr<ros::Subscriber>> subscribers_;

    // NRP: for gazebo based topics, track recording time for rate limiting
    boost::unordered_map<std::string, ros::Time> gazebo_topics_;
};

} // namespace rosbag

#endif
