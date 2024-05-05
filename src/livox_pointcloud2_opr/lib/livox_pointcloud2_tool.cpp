#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "pc2_sub_pub.h"

namespace livox_pc2_opr
{
    PointCloud2_SubPub::PointCloud2_SubPub(ros::NodeHandle *rosHandle) 
    {
        this->rosHandle = *rosHandle;
        init_subscribers();
        init_publishers();    
    }

    PointCloud2_SubPub::~PointCloud2_SubPub()
    {
    }

    void PointCloud2_SubPub::init_subscribers()
    {
        pc2Sub = rosHandle.subscribe(pointcloud2_topic, 10, &PointCloud2_SubPub::Pc2SubCallBack, this);
    }

    void PointCloud2_SubPub::init_publishers()
    {
        pc2Pub = rosHandle.advertise<sensor_msgs::PointCloud2>(pointcloud2_topic, 10);
    }

    

    void PointCloud2_SubPub::Pc2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &tempCloud)
    {
        pcl::fromROSMsg(*tempCloud, *receivedPointCloud);

        // ROS_INFO("received %ld points", receivedPointCloud->points.size());

    }

    void PointCloud2_SubPub::publish()
    {
        sensor_msgs::PointCloud2 msg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < 3; i++)
        {
            pcl::PointXYZI p;
            cloud->push_back(p);
        }

        pcl::toROSMsg(*cloud, msg);
        // or
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloud, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, msg);

        // publish
        pc2Pub.publish(msg);
        ROS_INFO("published.");
    } // namespace livox_pc2_opr

}



