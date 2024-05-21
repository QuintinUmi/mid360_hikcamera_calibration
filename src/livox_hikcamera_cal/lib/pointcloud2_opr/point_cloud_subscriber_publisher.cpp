#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"

namespace livox_hikcamera_cal::pointcloud2_opr
{
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher()
    {
        this->node_handle = ros::NodeHandle();
        this->subscribe_topic = std::string("/livox/lidar");
        this->publish_topic = std::string("/livox/lidar_proc");

        this->received_pcl_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        this->received_pcl_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        init_subscribers();
        init_publishers(); 
    }
    PointCloudSubscriberPublisher::PointCloudSubscriberPublisher(ros::NodeHandle node_handle, std::string subscribe_topic, std::string publish_topic)
    {
        this->node_handle = node_handle;
        this->subscribe_topic = subscribe_topic;
        this->publish_topic = publish_topic;
        
        this->received_pcl_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        this->received_pcl_xyzi = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        init_subscribers();
        init_publishers();    
    }

    PointCloudSubscriberPublisher::~PointCloudSubscriberPublisher()
    {
    }

    void PointCloudSubscriberPublisher::init_subscribers() 
    {
        pointcloud2_SUB = node_handle.subscribe<sensor_msgs::PointCloud2>(this->subscribe_topic, 10, &PointCloudSubscriberPublisher::PC2SubCallBack, this);
        printf("Init PointCloud2 Subscriber Success!\n");
    }

    void PointCloudSubscriberPublisher::init_publishers()
    {
        pointcloud2_PUB = node_handle.advertise<sensor_msgs::PointCloud2>(this->publish_topic, 10);
        printf("Init PointCloud2 Publisher Success!\n");
    }

    

    void PointCloudSubscriberPublisher::PC2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &rcvCloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*rcvCloud, *tempCloud);
        pcl::copyPointCloud(*tempCloud, *this->received_pcl_xyzi);
        pcl::copyPointCloud(*tempCloud, *this->received_pcl_xyz);
        // this->received_pcl_pointcloud->points[received_pcl_pointcloud->points.size()-1] = this->received_pcl_pointcloud->points[received_pcl_pointcloud->points.size()-2];
        
        // std::cout << this->received_pcl_pointcloud->points.size() << std::endl;
        // memccpy(&this->received_pcl_pointcloud, tempCloud, 1);
        // pcl::copyPointCloud()
        // std::cout << "height: " << received_pcl_pointcloud->height << " | width:" << received_pcl_pointcloud->width << std::endl;
        // ROS_INFO("received %ld points", received_pcl_pointcloud->points.size());

    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudSubscriberPublisher::getPointcloudXYZI()
    {
        return this->received_pcl_xyzi;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudSubscriberPublisher::getPointcloudXYZ()
    {
        return this->received_pcl_xyz;
    }


    void PointCloudSubscriberPublisher::publish(){}

    void PointCloudSubscriberPublisher::publish(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        sensor_msgs::PointCloud2 msg;

        pcl::toROSMsg(*cloud, msg);

        // publish
        pointcloud2_PUB.publish(msg);

    } 
    void PointCloudSubscriberPublisher::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        sensor_msgs::PointCloud2 msg;

        pcl::toROSMsg(*cloud, msg);

        // publish
        pointcloud2_PUB.publish(msg);

    } 

} // namespace livox_hikcamera_cal::pointcloud2_opr



