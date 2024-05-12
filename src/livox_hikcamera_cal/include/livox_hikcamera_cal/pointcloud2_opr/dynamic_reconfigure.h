#ifndef _DYNAMIC_RECONFIGURE_H_
#define _DYNAMIC_RECONFIGURE_H_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_hikcamera_cal/PointcloudFilterConfig.h>


namespace livox_hikcamera_cal
{
    namespace pointcloud2_opr
    {
        struct RQTConfig
        {
            struct _FilterConfig_
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

                RQTConfig::_FilterConfig_ getConfigure();

            private:
                void FilterReconfigureCallBack(livox_hikcamera_cal::PointcloudFilterConfig &pcFilterConfig, uint32_t level);

            private:
                RQTConfig::_FilterConfig_ filterCfg;
                dynamic_reconfigure::Server<livox_hikcamera_cal::PointcloudFilterConfig> server;
                dynamic_reconfigure::Server<livox_hikcamera_cal::PointcloudFilterConfig>::CallbackType f;
                
        };

        // class LidarReconfigure
        // {
        //     public:
        //         LidarReconfigure();

        //         void get_configure();

            
        // };

    }
}





#endif