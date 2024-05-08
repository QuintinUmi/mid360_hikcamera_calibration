#ifndef _POINT_CLOUD_PROCESS_H_
#define _POINT_CLOUD_PROCESS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "livox_pc2_opr/point_cloud_subscriber_publisher.h"

namespace livox_pc2_opr
{

    class PointCloud2Proc
    {
        public:
            PointCloud2Proc();
            PointCloud2Proc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
            ~PointCloud2Proc();

            void setCloud();
            void setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr get_raw_pointcloud();
            pcl::PointCloud<pcl::PointXYZI>::Ptr get_processed_pointcloud();

            PointCloud2Proc& filter(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

        private:
            pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr procCloud;

    };



}


#endif