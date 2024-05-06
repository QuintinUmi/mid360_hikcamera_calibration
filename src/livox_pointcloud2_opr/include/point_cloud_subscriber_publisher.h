#ifndef _POINT_CLOUD_SUBSCRIBER_PUBLISHER_H_
#define _POINT_CLOUD_SUBSCRIBER_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace livox_pc2_opr
{
    class PointCloudSubscriberPublisher
    {

        public:
            PointCloudSubscriberPublisher();
            PointCloudSubscriberPublisher(ros::NodeHandle rosHandle, std::string subscribe_topic, std::string publish_topic);
            ~PointCloudSubscriberPublisher();

            pcl::PointCloud<pcl::PointXYZI>::Ptr get_pointcloud();
            
            void publish();
            void publish(pcl::PointCloud<pcl::PointXYZI>::Ptr pubCloud);

        protected:
            pcl::PointCloud<pcl::PointXYZI>::Ptr receivedPointCloud;

        private:
            ros::NodeHandle rosHandle;
            ros::Subscriber pc2Sub;
            ros::Publisher pc2Pub;

            std::string subscribe_topic;
            std::string publish_topic;

            void init_subscribers();
            void init_publishers();
            void Pc2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud);

    };

}


#endif