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

namespace livox_hikcamera_cal
{

    class Recorder
    {

        public:
            enum class MessageType
            {
                PointCloud2,
                Image
            };

            class Topic
            {
                public:
                    Topic();
                    Topic(std::string topic, MessageType message_type);
                    ~Topic();

                    std::string topic;
                    MessageType message_type;
            };

        public:
            Recorder();
            Recorder(std::string savePath);
            Recorder(std::string savePath, Recorder::Topic topic);
            Recorder(std::string savePath, std::vector<Recorder::Topic> topics);
            Recorder(std::string savePath, std::string topic_str, Recorder::MessageType message_type);
            Recorder(std::string savePath, std::vector<std::string> topic_strs, std::vector<Recorder::MessageType> message_types);
            ~Recorder();

            void setSavePath(std::string savePath);
            void addTopic(Recorder::Topic topic);
            void setTopic(Recorder::Topic topic);
            void setTopic(std::vector<Recorder::Topic> topics);
            void addTopic(std::string topic_str, Recorder::MessageType message_type);
            void setTopic(std::string topic_str, Recorder::MessageType message_type);
            void setTopic(std::vector<std::string> topic_strs, std::vector<Recorder::MessageType> message_types);

            void startRecording();
            void stopRecording();

            void playRecording();

        private:
            std::string savePath_;
            std::vector<Recorder::Topic> topics_;

            ros::NodeHandle nh;
            std::vector<ros::Subscriber> pcSub;
            rosbag::Bag bag;
            int recorderStatus;

            void setRecorder();
            void resetRecorder();
            void resetTopics();
            void resetSavePath();
            void destroySubscirber();
            void closeBag();

            void PointCloud2CallBack(const sensor_msgs::PointCloud2ConstPtr& pcMsgs, const Recorder::Topic& topic);
            void ImageCallBack(const sensor_msgs::ImageConstPtr& imgMsgs, const Recorder::Topic& topic);
            std::string rosTimeToLocal(const ros::Time time);
    };
    
}




#endif