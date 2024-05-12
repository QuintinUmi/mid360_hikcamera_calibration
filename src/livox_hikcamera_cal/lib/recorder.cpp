#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <boost/bind.hpp>

#include "livox_hikcamera_cal/recorder.h"


namespace livox_hikcamera_cal
{
    Recorder::Topic::Topic(){}
    Recorder::Topic::Topic(std::string topic, MessageType message_type)
    {
        this->topic = topic;
        this->message_type = message_type;
    }
    Recorder::Topic::~Topic(){}


    Recorder::Recorder()
    {
        this->resetRecorder();
    }
    Recorder::Recorder(std::string savePath)
    {
        this->resetRecorder();
        this->savePath_ = savePath;
    }
    Recorder::Recorder(std::string savePath, Recorder::Topic topic)
    {
        this->resetRecorder();
        this->savePath_ = savePath;
        this->topics_.emplace_back(topic);
        this->setRecorder();
    }
    Recorder::Recorder(std::string savePath, std::vector<Recorder::Topic> topics)
    {
        this->resetRecorder();
        this->savePath_ = savePath;
        this->topics_ = topics;
        this->setRecorder();
    }
    Recorder::Recorder(std::string savePath, std::string topic_str, Recorder::MessageType message_type)
    {
        this->resetRecorder();
        this->savePath_ = savePath;
        this->topics_.emplace_back(Recorder::Topic(topic_str, message_type));
        this->setRecorder();
    }
    Recorder::Recorder(std::string savePath, std::vector<std::string> topic_strs, std::vector<Recorder::MessageType> message_types)
    {
        this->resetRecorder();
        this->savePath_ = savePath;
        for(int i = 0; i < topic_strs.size(); i++)
        {
            this->topics_.emplace_back(Recorder::Topic(topic_strs[i], message_types[i]));
        }
        this->setRecorder();
    }

    Recorder::~Recorder()
    {
        this->resetRecorder();
    }


    void Recorder::setSavePath(std::string savePath)
    {
        this->destroySubscirber();
        this->resetSavePath();
        this->savePath_ = savePath;
        this->setRecorder();
    }
    void Recorder::addTopic(Recorder::Topic topic)
    {
        this->destroySubscirber();
        this->topics_.emplace_back(topic);
        this->setRecorder();
    }
    void Recorder::setTopic(Recorder::Topic topic)
    {
        this->resetRecorder();
        this->topics_.emplace_back(topic);
        this->setRecorder();
    }
    void Recorder::setTopic(std::vector<Recorder::Topic> topics)
    {
        this->resetRecorder();
        this->topics_ = topics;
        this->setRecorder();
    }
    void Recorder::addTopic(std::string topic_str, Recorder::MessageType message_type)
    {
        this->destroySubscirber();
        this->topics_.emplace_back(Recorder::Topic(topic_str, message_type));
        this->setRecorder();
    }
    void Recorder::setTopic(std::string topic_str, Recorder::MessageType message_type)
    {
        this->resetRecorder();
        this->topics_.emplace_back(Recorder::Topic(topic_str, message_type));
        this->setRecorder();
    }
    void Recorder::setTopic(std::vector<std::string> topic_strs, std::vector<Recorder::MessageType> message_types)
    {
        this->resetRecorder();
        for(int i = 0; i < topic_strs.size(); i++)
        {
            this->topics_.emplace_back(Recorder::Topic(topic_strs[i], message_types[i]));
        }
        this->setRecorder();
    }

    void Recorder::startRecording()
    {
        if(this->topics_.empty())
        {
            printf("Please set the subscriber topic first!!!\n");
            return;
        }
        if(this->recorderStatus == 1)
        {
            printf("Rosbag has been recording!!!\n");
            return;
        }

        std::string saveFilePath;
        if(!boost::iequals(this->savePath_.substr(this->savePath_.length() - 4), std::string(".bag")))
        {
            if(!boost::iequals(this->savePath_.substr(this->savePath_.length() - 1), std::string("/")))
            {
                this->savePath_ = this->savePath_ + std::string("/");
            }
            
            saveFilePath = this->savePath_ + this->rosTimeToLocal(ros::Time::now()) + std::string(".bag");
        }
        else
        {
            saveFilePath = this->savePath_;
        }

        this->bag.open(saveFilePath, rosbag::bagmode::Write);
        this->recorderStatus = 1;
    }
    void Recorder::stopRecording()
    {
        this->recorderStatus = 0;
        this->closeBag();
    }

    void Recorder::playRecording()
    {
        
    }



    void Recorder::setRecorder()
    {
        this->destroySubscirber();

        ros::Subscriber subTemp;
        
        for(auto iterTopic:this->topics_)
        {
            switch (iterTopic.message_type) {
                case Recorder::MessageType::PointCloud2:
                    subTemp = this->nh.subscribe<sensor_msgs::PointCloud2>(iterTopic.topic, 10, boost::bind(&Recorder::PointCloud2CallBack, this, _1, iterTopic));
                    this->pcSub.emplace_back(subTemp);
                    break;
                case Recorder::MessageType::Image:
                    subTemp = this->nh.subscribe<sensor_msgs::Image>(iterTopic.topic, 10, boost::bind(&Recorder::ImageCallBack, this, _1, iterTopic));
                    this->pcSub.emplace_back(subTemp);
                    break;
                default:
                    std::cerr << "Unknown message type" << std::endl;
            }
            
        }
        
    }
    void Recorder::resetRecorder()
    {
        this->destroySubscirber(); 
        this->resetTopics();
        this->resetSavePath();
    }
    void Recorder::resetTopics()
    {
        this->recorderStatus = 0;
        this->topics_.clear();
    }
    void Recorder::resetSavePath()
    {
        this->recorderStatus = 0;
        this->savePath_.clear();
    }
    void Recorder::destroySubscirber()
    {
        this->recorderStatus = 0;
        this->closeBag();
        for(auto iterSub:this->pcSub)
        {
            iterSub.shutdown();
        }
        this->pcSub.clear();
    }
    void Recorder::closeBag()
    {
        this->recorderStatus = 0;
        if(this->bag.isOpen())
        {
            this->bag.close();
        }
    }



    void Recorder::PointCloud2CallBack(const sensor_msgs::PointCloud2ConstPtr& pcMsgs, const Recorder::Topic& topic)
    {
        if(this->recorderStatus == 1)
        {
            this->bag.write(topic.topic, ros::Time::now(), pcMsgs);
            // this->bag.write(topic.topic, pcMsgs->header.stamp, pcMsgs);
        }
    }
    void Recorder::ImageCallBack(const sensor_msgs::ImageConstPtr& imgMsgs, const Recorder::Topic& topic)
    {
        if(this->recorderStatus == 1)
        {
            this->bag.write(topic.topic, ros::Time::now(), imgMsgs);
            // this->bag.write(topic.topic, imgMsgs->header.stamp, imgMsgs);
        }
    }

    std::string Recorder::rosTimeToLocal(const ros::Time time) 
    {
        const std::time_t time_c = time.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }
}

