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
        this->reset_recorder();
    }
    Recorder::Recorder(std::string savePath)
    {
        this->reset_recorder();
        this->savePath = savePath;
    }
    Recorder::Recorder(std::string savePath, std::string recordTopic)
    {
        this->reset_recorder();
        this->savePath = savePath;
        this->recordTopics.emplace_back(recordTopic);
        this->set_recorder();
    }
    Recorder::Recorder(std::string savePath, std::vector<std::string> recordTopics)
    {
        this->reset_recorder();
        this->savePath = savePath;
        this->recordTopics = recordTopics;
        this->set_recorder();
    }

    Recorder::~Recorder()
    {
        this->reset_recorder();
    }

    void Recorder::set_save_path(std::string savePath)
    {
        this->destroy_subscirber();
        this->reset_save_path();
        this->savePath = savePath;
        
        this->set_recorder();
    }
    void Recorder::set_record_topic(std::string recordTopic)
    {
        this->destroy_subscirber();
        this->reset_record_topics();
        this->recordTopics.emplace_back(recordTopic);
        this->set_recorder();
    }
    void Recorder::set_record_topic(std::vector<std::string> recordTopics)
    {
        this->recordTopics = recordTopics;
    }

    void Recorder::start_recording()
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
            
            saveFilePath = this->savePath + this->ros_time_to_local(ros::Time::now());
        }
        else
        {
            saveFilePath = this->savePath;
        }

        this->bag.open(saveFilePath, rosbag::bagmode::Write);
        this->recorderStatus = 1;
    }
    void Recorder::stop_recording()
    {
        this->recorderStatus = 0;
        this->close_bag();
    }

    void Recorder::play_recording()
    {
        
    }



    void Recorder::set_recorder()
    {
        this->destroy_subscirber();

        ros::Subscriber subTemp;
        
        for(auto iterTopic:this->recordTopics)
        {
            subTemp = this->nh.subscribe<sensor_msgs::PointCloud2>(iterTopic, 10, boost::bind(&Recorder::RecordingCallBack, this, _1, iterTopic));
            this->pcSub.emplace_back(subTemp);
        }
        
    }
    void Recorder::reset_recorder()
    {
        this->destroy_subscirber(); 
        this->reset_record_topics();
        this->savePath.clear();
        this->recorderStatus = 0;
    }
    void Recorder::reset_record_topics()
    {
        this->recordTopics.clear();
    }
    void Recorder::reset_save_path()
    {
        this->savePath.clear();
    }
    void Recorder::destroy_subscirber()
    {
        this->close_bag();
        for(auto iterSub:this->pcSub)
        {
            iterSub.shutdown();
        }
        this->pcSub.clear();
    }
    void Recorder::close_bag()
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

    std::string Recorder::ros_time_to_local(const ros::Time time) 
    {
        const std::time_t time_c = time.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }
}

