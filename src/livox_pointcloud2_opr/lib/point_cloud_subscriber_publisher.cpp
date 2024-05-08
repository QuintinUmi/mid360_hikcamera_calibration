#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "livox_pc2_opr/point_cloud_subscriber_publisher.h"

namespace livox_pc2_opr
{
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher()
    {
        this->rosHandle = ros::NodeHandle();
        this->subscribe_topic = std::string("/livox/lidar");
        this->publish_topic = std::string("/livox/lidar_proc");

        this->receivedPCLPointCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        init_subscribers();
        init_publishers(); 
    }
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher(ros::NodeHandle rosHandle, std::string subscribe_topic, std::string publish_topic)
    {
        this->rosHandle = rosHandle;
        this->subscribe_topic = subscribe_topic;
        this->publish_topic = publish_topic;
        
        this->receivedPCLPointCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        init_subscribers();
        init_publishers();    
    }

    PointCloudSubscriberPublisher::~PointCloudSubscriberPublisher()
    {
    }

    void PointCloudSubscriberPublisher::init_subscribers() 
    {
        pc2Sub = rosHandle.subscribe<sensor_msgs::PointCloud2>(this->subscribe_topic, 10, &PointCloudSubscriberPublisher::Pc2SubCallBack, this);
    }

    void PointCloudSubscriberPublisher::init_publishers()
    {
        pc2Pub = rosHandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic, 10);
    }

    

    void PointCloudSubscriberPublisher::Pc2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &rcvCloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rcvCloud, *tempCloud);
        pcl::copyPointCloud(*tempCloud, *this->receivedPCLPointCloud);
        // this->receivedPCLPointCloud->points[receivedPCLPointCloud->points.size()-1] = this->receivedPCLPointCloud->points[receivedPCLPointCloud->points.size()-2];
        
        // std::cout << this->receivedPCLPointCloud->points.size() << std::endl;
        // memccpy(&this->receivedPCLPointCloud, tempCloud, 1);
        // pcl::copyPointCloud()
        // std::cout << "height: " << receivedPCLPointCloud->height << " | width:" << receivedPCLPointCloud->width << std::endl;
        // ROS_INFO("received %ld points", receivedPCLPointCloud->points.size());

    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudSubscriberPublisher::get_pointcloud()
    {
        return this->receivedPCLPointCloud;
    }


    void PointCloudSubscriberPublisher::publish(){}

    void PointCloudSubscriberPublisher::publish(pcl::PointCloud<pcl::PointXYZI>::Ptr pubCloud)
    {
        sensor_msgs::PointCloud2 msg;

        pcl::toROSMsg(*pubCloud, msg);

        // publish
        pc2Pub.publish(msg);

    } 

} // namespace livox_pc2_opr



