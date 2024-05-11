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

            void setSavePath(std::string savePath);
            void setRecordTopic(std::string recordTopic);
            void setRecordTopic(std::vector<std::string> recordTopics);

            void startRecording();
            void stopRecording();

            void playRecording();

        private:
            std::string savePath;
            std::vector<std::string> recordTopics;

            ros::NodeHandle nh;
            std::vector<ros::Subscriber> pcSub;
            rosbag::Bag bag;
            int recorderStatus;

            void setRecorder();
            void resetRecorder();
            void resetRecordTopics();
            void resetSavePath();
            void destroySubscirber();
            void closeBag();

            void RecordingCallBack(const sensor_msgs::PointCloud2ConstPtr& pcMsgs, const std::string& topic);
            std::string rosTimeToLocal(const ros::Time time);
    };
    
}


#endif