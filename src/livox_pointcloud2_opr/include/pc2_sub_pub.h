#ifndef _PC2_RECEIVER_H_
#define _PC2_RECEIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace livox_pc2_opr
{
    class PointCloud2_SubPub
    {

    public:
        PointCloud2_SubPub(ros::NodeHandle *rosHandle);
        ~PointCloud2_SubPub();
        void publish();

    public:
        pcl::PointCloud<pcl::PointXYZI>::Ptr receivedPointCloud;

    private:
        ros::NodeHandle rosHandle;
        ros::Subscriber pc2Sub;
        ros::Publisher pc2Pub;

        std::string pointcloud2_topic;

        void init_subscribers();
        void init_publishers();
        void Pc2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud);

    
    };

}


#endif