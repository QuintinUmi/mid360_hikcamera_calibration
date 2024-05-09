#ifndef _RECORDER_H_
#define _RECORDER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <rosbag/bag.h>
#include <boost/bind.hpp>

namespace livox_pc2_opr
{
    class Recorder
    {
        public:
            Recorder();
            Recorder(std::string savePath);
            Recorder(std::string savePath, std::string recordTopic);
            Recorder(std::string savePath, std::vector<std::string> recordTopics);
            ~Recorder();

            void set_save_path(std::string savePath);
            void set_record_topic(std::string recordTopic);
            void set_record_topic(std::vector<std::string> recordTopics);

            void start_recording();
            void stop_recording();

            void play_recording();

        private:
            std::string savePath;
            std::vector<std::string> recordTopics;

            ros::NodeHandle nh;
            std::vector<ros::Subscriber> pcSub;
            rosbag::Bag bag;
            int recorderStatus;

            void set_recorder();
            void reset_recorder();
            void reset_record_topics();
            void reset_save_path();
            void destroy_subscirber();
            void close_bag();

            void RecordingCallBack(const sensor_msgs::PointCloud2ConstPtr& pcMsgs, const std::string& topic);
            std::string ros_time_to_local(const ros::Time time);
    };
    
}


#endif