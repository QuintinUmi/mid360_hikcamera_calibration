#ifndef _PCD_SAVER_H_
#define _PCD_SAVER_H_

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
        
        class PCDSaver
        {
            public:
                PCDSaver();
                PCDSaver(std::string save_path);

                void setSavePath(std::string save_path);

                template<typename PointT>
                void save(sensor_msgs::PointCloud2ConstPtr cloud);

                template<typename PointT>
                void save(typename pcl::PointCloud<PointT>::Ptr cloud);


            private:
                std::string save_path;
                

                std::string fileNameGenerator(sensor_msgs::PointCloud2ConstPtr cloud);
                std::string fileNameGenerator(sensor_msgs::PointCloud2 cloud);
                std::string frameTimeToLocal(sensor_msgs::PointCloud2ConstPtr cloud);
                std::string frameTimeToLocal(sensor_msgs::PointCloud2 cloud);
                

        };



        template<typename PointT>
        void PCDSaver::save(sensor_msgs::PointCloud2ConstPtr cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(*cloud, *pclCloud);

            pcl::io::savePCDFileASCII(this->fileNameGenerator(cloud), *pclCloud);
            ROS_INFO("Saved cloud with %ld points", pclCloud->points.size());
        }
        template<typename PointT>
        void PCDSaver::save(typename pcl::PointCloud<PointT>::Ptr pclCloud)
        {
            
            sensor_msgs::PointCloud2 rosCloud;
            
            pcl::toROSMsg(*pclCloud, rosCloud);   
            pcl::io::savePCDFileASCII(this->fileNameGenerator(rosCloud), *pclCloud);
            
            ROS_INFO("Saved cloud with %ld points", pclCloud->points.size());
        }
    
    }
}






#endif