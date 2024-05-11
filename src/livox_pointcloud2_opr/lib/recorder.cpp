#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <boost/bind.hpp>

#include "livox_pc2_opr/recorder.h"


namespace livox_pc2_opr
{
    Recorder::Recorder()
    {
        this->resetRecorder();
    }
    Recorder::Recorder(std::string savePath)
    {
        this->resetRecorder();
        this->savePath = savePath;
    }
    Recorder::Recorder(std::string savePath, std::string recordTopic)
    {
        this->resetRecorder();
        this->savePath = savePath;
        this->recordTopics.emplace_back(recordTopic);
        this->setRecorder();
    }
    Recorder::Recorder(std::string savePath, std::vector<std::string> recordTopics)
    {
        this->resetRecorder();
        this->savePath = savePath;
        this->recordTopics = recordTopics;
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
        this->savePath = savePath;
        
        this->setRecorder();
    }
    void Recorder::setRecordTopic(std::string recordTopic)
    {
        this->destroySubscirber();
        this->resetRecordTopics();
        this->recordTopics.emplace_back(recordTopic);
        this->setRecorder();
    }
    void Recorder::setRecordTopic(std::vector<std::string> recordTopics)
    {
        this->recordTopics = recordTopics;
    }

    void Recorder::startRecording()
    {
        if(this->recordTopics.empty())
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
        if(!boost::iequals(savePath.substr(savePath.length() - 4), std::string(".bag")))
        {
            if(!boost::iequals(savePath.substr(savePath.length() - 1), std::string("/")))
            {
                this->savePath = this->savePath + std::string("/");
            }
            
            saveFilePath = this->savePath + this->rosTimeToLocal(ros::Time::now()) + std::string(".bag");
        }
        else
        {
            saveFilePath = this->savePath;
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
        
        for(auto iterTopic:this->recordTopics)
        {
            subTemp = this->nh.subscribe<sensor_msgs::PointCloud2>(iterTopic, 10, boost::bind(&Recorder::RecordingCallBack, this, _1, iterTopic));
            this->pcSub.emplace_back(subTemp);
        }
        
    }
    void Recorder::resetRecorder()
    {
        this->destroySubscirber(); 
        this->resetRecordTopics();
        this->savePath.clear();
        this->recorderStatus = 0;
    }
    void Recorder::resetRecordTopics()
    {
        this->recordTopics.clear();
    }
    void Recorder::resetSavePath()
    {
        this->savePath.clear();
    }
    void Recorder::destroySubscirber()
    {
        this->closeBag();
        for(auto iterSub:this->pcSub)
        {
            iterSub.shutdown();
        }
        this->pcSub.clear();
    }
    void Recorder::closeBag()
    {
        if(this->bag.isOpen())
        {
            this->bag.close();
        }
    }



    void Recorder::RecordingCallBack(const sensor_msgs::PointCloud2ConstPtr& pcMsgs, const std::string& topic)
    {
        if(this->recorderStatus == 1)
        {
            this->bag.write(topic, pcMsgs->header.stamp, pcMsgs);
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

