#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/bind.hpp>

#include "livox_pc2_opr/pcd_saver.h"

namespace livox_pc2_opr
{
    PCDSaver::PCDSaver()
    {
    }
    PCDSaver::PCDSaver(std::string savePath)
    {
        this->savePath = savePath;
    }

    void PCDSaver::set_save_path(std::string savePath)
    {
        this->savePath = savePath;
    }

    void PCDSaver::save(sensor_msgs::PointCloud2ConstPtr cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud, *pclCloud);

        pcl::io::savePCDFileASCII(this->file_name_generator(cloud), *pclCloud);
        ROS_INFO("Saved cloud with %ld points", pclCloud->points.size());
    }
    void PCDSaver::save(pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud)
    {
        
        sensor_msgs::PointCloud2 rosCloud;
        
        pcl::toROSMsg(*pclCloud, rosCloud);   
        pcl::io::savePCDFileASCII(this->file_name_generator(rosCloud), *pclCloud);
        
        ROS_INFO("Saved cloud with %ld points", pclCloud->points.size());
    }




    std::string PCDSaver::file_name_generator(sensor_msgs::PointCloud2ConstPtr cloud)
    {
        std::string saveFilePath;
        if(!boost::iequals(this->savePath.substr(this->savePath.length() - 4), std::string(".pcd")))
        {
            if(!boost::iequals(this->savePath.substr(this->savePath.length() - 1), std::string("/")))
            {
                this->savePath = this->savePath + std::string("/");
            }
            
            saveFilePath = this->savePath + this->frame_time_to_local(cloud) + std::string(".pcd");
        }
        else
        {
            saveFilePath = this->savePath;
        }

        return saveFilePath;
    }
    std::string PCDSaver::file_name_generator(sensor_msgs::PointCloud2 cloud)
    {
        std::string saveFilePath;
        if(!boost::iequals(this->savePath.substr(this->savePath.length() - 4), std::string(".pcd")))
        {
            if(!boost::iequals(this->savePath.substr(this->savePath.length() - 1), std::string("/")))
            {
                this->savePath = this->savePath + std::string("/");
            }
            // printf("test-------------------------------------------\n");
            saveFilePath = this->savePath + this->frame_time_to_local(cloud) + std::string(".pcd");
            std::cout << saveFilePath << std::endl;
        }
        else
        {
            saveFilePath = this->savePath;
        }

        return saveFilePath;
    }


    std::string PCDSaver::frame_time_to_local(sensor_msgs::PointCloud2ConstPtr cloud)
    {
        const ros::Time frameTime = cloud->header.stamp;
        const std::time_t time_c = frameTime.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }
    std::string PCDSaver::frame_time_to_local(sensor_msgs::PointCloud2 cloud)
    {
        const ros::Time frameTime = cloud.header.stamp;
        const std::time_t time_c = frameTime.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }

}