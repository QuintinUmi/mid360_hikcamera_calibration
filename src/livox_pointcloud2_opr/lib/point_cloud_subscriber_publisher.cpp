#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "point_cloud_subscriber_publisher.h"

namespace livox_pc2_opr
{
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher()
    {
        this->rosHandle = ros::NodeHandle();
        this->subscribe_topic = std::string("/livox/lidar");
        this->publish_topic = std::string("/livox/lidar_proc");

        this->receivedPointCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        init_subscribers();
        init_publishers(); 
    }
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher(ros::NodeHandle rosHandle, std::string subscribe_topic, std::string publish_topic)
    {
        this->rosHandle = rosHandle;
        this->subscribe_topic = subscribe_topic;
        this->publish_topic = publish_topic;
        
        this->receivedPointCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

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
        pcl::copyPointCloud(*tempCloud, *this->receivedPointCloud);
        
        // std::cout << this->receivedPointCloud->points.size() << std::endl;
        // memccpy(&this->receivedPointCloud, tempCloud, 1);
        // pcl::copyPointCloud()
        // std::cout << "height: " << receivedPointCloud->height << " | width:" << receivedPointCloud->width << std::endl;
        // ROS_INFO("received %ld points", receivedPointCloud->points.size());

    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudSubscriberPublisher::get_pointcloud()
    {
        return this->receivedPointCloud;
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



