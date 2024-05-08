#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>

#include "livox_pc2_opr/point_cloud_subscriber_publisher.h"
#include "livox_pc2_opr/point_cloud_process.h"

namespace livox_pc2_opr
{
    PointCloud2Proc::PointCloud2Proc() :    rawCloud(new pcl::PointCloud<pcl::PointXYZI>),
                                            procCloud(new pcl::PointCloud<pcl::PointXYZI>) {};
    
    PointCloud2Proc::PointCloud2Proc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) :  rawCloud(cloud),
                                                                                    procCloud(new pcl::PointCloud<pcl::PointXYZI>) 
    {
        pcl::copyPointCloud(*rawCloud, *procCloud);
    }

    PointCloud2Proc::~PointCloud2Proc() {}

    void PointCloud2Proc::setCloud()
    {
        if(rawCloud){
            printf("Point Cloud Re-setting Failed! Not allow empty point cloud reset");
            return;
        }
        pcl::copyPointCloud(*rawCloud, *procCloud);
    }
    void PointCloud2Proc::setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::copyPointCloud(*cloud, *rawCloud);
        pcl::copyPointCloud(*cloud, *procCloud);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::get_raw_pointcloud()
    {
        return this->rawCloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::get_processed_pointcloud()
    {
        return this->procCloud;
    }

    PointCloud2Proc& PointCloud2Proc::filter(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
    {
        pcl::CropBox<pcl::PointXYZI> boxFilter;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
        boxFilter.setInputCloud(this->procCloud);
        boxFilter.setMin(minPoint);
        boxFilter.setMax(maxPoint);

        boxFilter.filter(*tempCloud);
        pcl::copyPointCloud(*tempCloud, *this->procCloud);
        
        return *this;
    }

}