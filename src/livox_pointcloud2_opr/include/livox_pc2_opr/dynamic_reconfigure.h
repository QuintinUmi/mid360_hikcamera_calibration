#ifndef _DYNAMIC_RECONFIGURE_H_
#define _DYNAMIC_RECONFIGURE_H_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_pointcloud2_opr/PointcloudFilterConfig.h>


namespace livox_pc2_opr
{
    struct RQTConfig
    {
        struct FilterConfig_
        {
            float x_min; 
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
        }FilterConfig;

        // struct LidarConfig
        // {
        //     float a;
        //     float b;
        // };
        
    };

    class PointcloudFilterReconfigure 
    {
        public:
            PointcloudFilterReconfigure();

            RQTConfig::FilterConfig_ get_configure();

        private:
            void FilterReconfigure_CallBack(livox_pointcloud2_opr::PointcloudFilterConfig &pcFilterConfig, uint32_t level);

        private:
            RQTConfig::FilterConfig_ filterCfg;
            dynamic_reconfigure::Server<livox_pointcloud2_opr::PointcloudFilterConfig> server;
            dynamic_reconfigure::Server<livox_pointcloud2_opr::PointcloudFilterConfig>::CallbackType f;
            
    };

    // class LidarReconfigure
    // {
    //     public:
    //         LidarReconfigure();

    //         void get_configure();

        
    // };

}



#endif