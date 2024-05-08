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
            PCDSaver(std::string savePath);

            void set_save_path(std::string savePath);

            void save(sensor_msgs::PointCloud2ConstPtr cloud);
            void save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);


        private:
            std::string savePath;




            std::string file_name_generator(sensor_msgs::PointCloud2ConstPtr cloud);
            std::string file_name_generator(sensor_msgs::PointCloud2 cloud);
            std::string frame_time_to_local(sensor_msgs::PointCloud2ConstPtr cloud);
            std::string frame_time_to_local(sensor_msgs::PointCloud2 cloud);
            

    };
}


#endif