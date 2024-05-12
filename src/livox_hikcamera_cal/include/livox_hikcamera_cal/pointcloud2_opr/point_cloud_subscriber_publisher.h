#ifndef _POINT_CLOUD_SUBSCRIBER_PUBLISHER_H_
#define _POINT_CLOUD_SUBSCRIBER_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


namespace livox_hikcamera_cal
{

    namespace pointcloud2_opr
    {
        class PointCloudSubscriberPublisher
        {

            public:
                PointCloudSubscriberPublisher();
                PointCloudSubscriberPublisher(ros::NodeHandle node_handle, std::string subscribe_topic, std::string publish_topic);
                ~PointCloudSubscriberPublisher();

                pcl::PointCloud<pcl::PointXYZI>::Ptr getPointcloudXYZI();
                pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloudXYZ();
                
                void publish();
                void publish(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
                void publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

            protected:
                pcl::PointCloud<pcl::PointXYZI>::Ptr received_pcl_xyzi;
                pcl::PointCloud<pcl::PointXYZ>::Ptr received_pcl_xyz;

            private:
                ros::NodeHandle node_handle;
                ros::Subscriber pointcloud2_SUB;
                ros::Publisher pointcloud2_PUB;

                std::string subscribe_topic;
                std::string publish_topic;

                void init_subscribers();
                void init_publishers();
                void PC2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud);

        };

    }

}



#endif