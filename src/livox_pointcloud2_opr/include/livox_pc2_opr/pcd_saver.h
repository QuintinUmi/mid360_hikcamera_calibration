#ifndef _PCD_SAVER_H_
#define _PCD_SAVER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


namespace livox_pc2_opr
{
    class PCDSaver
    {
        public:
            PCDSaver();
            PCDSaver(std::string save_path);

            void setSavePath(std::string save_path);

            void save(sensor_msgs::PointCloud2ConstPtr cloud);
            void save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);


        private:
            std::string save_path;




            std::string fileNameGenerator(sensor_msgs::PointCloud2ConstPtr cloud);
            std::string fileNameGenerator(sensor_msgs::PointCloud2 cloud);
            std::string frameTimeToLocal(sensor_msgs::PointCloud2ConstPtr cloud);
            std::string frameTimeToLocal(sensor_msgs::PointCloud2 cloud);
            

    };
}


#endif