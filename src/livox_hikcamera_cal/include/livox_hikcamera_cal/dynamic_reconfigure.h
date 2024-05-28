#ifndef _DYNAMIC_RECONFIGURE_H_
#define _DYNAMIC_RECONFIGURE_H_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_hikcamera_cal/OrthoFilterConfig.h>
#include <livox_hikcamera_cal/TransformFilterConfig.h>
#include <livox_hikcamera_cal/CalibrationParamConfig.h>


namespace livox_hikcamera_cal
{
    struct RQTConfig
    {
        struct _OrthoFilterConfig_
        {
            float x_min; 
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
        }OrthoFilterConfig;

        struct _TransformFilterConfig_
        {
            float center_x;
            float center_y;
            float center_z;
            float length_x;
            float length_y;
            float length_z;
            float rotate_x;
            float rotate_y;
            float rotate_z;
        }TransformFilterConfig;

        struct _CalibrationParamConfig_
        {
            double concave_hull_alpha;
        }CalibrationParam;

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
            PointcloudFilterReconfigure(ros::NodeHandle nh);

            RQTConfig::_OrthoFilterConfig_ getOrthoConfigure();
            RQTConfig::_TransformFilterConfig_ getTransformConfigure();
            bool isUpdated();

        private:

            void OrthoFilterReconfigureCallBack(livox_hikcamera_cal::OrthoFilterConfig &pcOrthoFilterConfig, uint32_t level);
            void TransformFilterReconfigureCallBack(livox_hikcamera_cal::TransformFilterConfig &pcTransformFilterConfig, uint32_t level);

        private:

            RQTConfig::_OrthoFilterConfig_ OrthoFilterCfg;
            RQTConfig::_TransformFilterConfig_ TransformFilterConfig;
            dynamic_reconfigure::Server<livox_hikcamera_cal::OrthoFilterConfig> ortho_filter_server;
            dynamic_reconfigure::Server<livox_hikcamera_cal::OrthoFilterConfig>::CallbackType ortho_filter_f;
            dynamic_reconfigure::Server<livox_hikcamera_cal::TransformFilterConfig> transform_filter_server;
            dynamic_reconfigure::Server<livox_hikcamera_cal::TransformFilterConfig>::CallbackType transform_filter_f;

            bool is_updated_ = false;
            
    };

    class CalibrationParamReconfigure 
    {
        public:

            CalibrationParamReconfigure();
            CalibrationParamReconfigure(ros::NodeHandle nh);

            RQTConfig::_CalibrationParamConfig_ getCalibrationParamConfigure();
            bool isUpdated();

        private:

            void CalibrationParamReconfigureCallBack(livox_hikcamera_cal::CalibrationParamConfig& calibrationParamConfig, uint32_t level);

        private:

            RQTConfig::_CalibrationParamConfig_ CalibrationParamCfg;
            dynamic_reconfigure::Server<livox_hikcamera_cal::CalibrationParamConfig> calibration_param_server;
            dynamic_reconfigure::Server<livox_hikcamera_cal::CalibrationParamConfig>::CallbackType calibration_param_f;

            bool is_updated_ = false;
            
    };

    // class LidarReconfigure
    // {
    //     public:
    //         LidarReconfigure();

    //         void get_configure();

        
    // };
    
}





#endif