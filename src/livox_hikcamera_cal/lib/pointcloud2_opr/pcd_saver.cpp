#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/bind.hpp>

#include "livox_hikcamera_cal/pointcloud2_opr/pcd_saver.h"

namespace livox_hikcamera_cal::pointcloud2_opr
{
    PCDSaver::PCDSaver()
    {
    }
    PCDSaver::PCDSaver(std::string save_path)
    {
        this->save_path = save_path;
    }

    void PCDSaver::setSavePath(std::string save_path)
    {
        this->save_path = save_path;
    }
    



    std::string PCDSaver::fileNameGenerator(sensor_msgs::PointCloud2ConstPtr cloud)
    {
        std::string saveFilePath;
        if(!boost::iequals(this->save_path.substr(this->save_path.length() - 4), std::string(".pcd")))
        {
            if(!boost::iequals(this->save_path.substr(this->save_path.length() - 1), std::string("/")))
            {
                this->save_path = this->save_path + std::string("/");
            }
            
            saveFilePath = this->save_path + this->frameTimeToLocal(cloud) + std::string(".pcd");
        }
        else
        {
            saveFilePath = this->save_path;
        }

        return saveFilePath;
    }
    std::string PCDSaver::fileNameGenerator(sensor_msgs::PointCloud2 cloud)
    {
        std::string saveFilePath;
        if(!boost::iequals(this->save_path.substr(this->save_path.length() - 4), std::string(".pcd")))
        {
            if(!boost::iequals(this->save_path.substr(this->save_path.length() - 1), std::string("/")))
            {
                this->save_path = this->save_path + std::string("/");
            }
            // printf("test-------------------------------------------\n");
            saveFilePath = this->save_path + this->frameTimeToLocal(cloud) + std::string(".pcd");
            std::cout << saveFilePath << std::endl;
        }
        else
        {
            saveFilePath = this->save_path;
        }

        return saveFilePath;
    }


    std::string PCDSaver::frameTimeToLocal(sensor_msgs::PointCloud2ConstPtr cloud)
    {
        const ros::Time frameTime = cloud->header.stamp;
        const std::time_t time_c = frameTime.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }
    std::string PCDSaver::frameTimeToLocal(sensor_msgs::PointCloud2 cloud)
    {
        const ros::Time frameTime = cloud.header.stamp;
        const std::time_t time_c = frameTime.sec; 
        std::tm* tm = std::localtime(&time_c); 
        std::stringstream ss;
        ss << std::put_time(tm, "%Y%m%d%H%M%S");
        return ss.str();
    }

}