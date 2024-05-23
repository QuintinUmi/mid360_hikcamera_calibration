#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_hikcamera_cal/OrthoFilterConfig.h>
#include <livox_hikcamera_cal/TransformFilterConfig.h>

#include "livox_hikcamera_cal/dynamic_reconfigure.h"

using namespace livox_hikcamera_cal;


PointcloudFilterReconfigure::PointcloudFilterReconfigure() :    ortho_filter_server(ros::NodeHandle("OrthoFilterReconfigure")),
                                                                transform_filter_server(ros::NodeHandle("TransformFilterReconfigure"))
{

    this->ortho_filter_f = boost::bind(&PointcloudFilterReconfigure::OrthoFilterReconfigureCallBack, this, _1, _2);
    this->ortho_filter_server.setCallback(this->ortho_filter_f);

    this->transform_filter_f = boost::bind(&PointcloudFilterReconfigure::TransformFilterReconfigureCallBack, this, _1, _2);
    this->transform_filter_server.setCallback(this->transform_filter_f);
    
}

void PointcloudFilterReconfigure::OrthoFilterReconfigureCallBack(livox_hikcamera_cal::OrthoFilterConfig &pcOrthoFilterConfig, uint32_t level) 
{
    ROS_INFO("Ortho Filter Reconfigure: x_min=%f, x_max=%f, y_min=%f, y_max=%f, z_min=%f, z_max=%f",
            pcOrthoFilterConfig.x_min, pcOrthoFilterConfig.x_max, pcOrthoFilterConfig.y_min, pcOrthoFilterConfig.y_max, pcOrthoFilterConfig.z_min, pcOrthoFilterConfig.z_max);
    
    this->OrthoFilterCfg.x_min = pcOrthoFilterConfig.x_min;
    this->OrthoFilterCfg.x_max = pcOrthoFilterConfig.x_max;
    this->OrthoFilterCfg.y_min = pcOrthoFilterConfig.y_min;
    this->OrthoFilterCfg.y_max = pcOrthoFilterConfig.y_max;
    this->OrthoFilterCfg.z_min = pcOrthoFilterConfig.z_min;
    this->OrthoFilterCfg.z_max = pcOrthoFilterConfig.z_max;

    this->is_updated_ = true;
}
void PointcloudFilterReconfigure::TransformFilterReconfigureCallBack(livox_hikcamera_cal::TransformFilterConfig &pcTransformFilterConfig, uint32_t level) 
{
    ROS_INFO("Transform Filter Reconfigure: center_x=%f, center_y=%f, center_z=%f, length_x=%f, length_y=%f, length_z=%f, rotate_x=%f, rotate_y=%f, rotate_z=%f",
            pcTransformFilterConfig.center_x, pcTransformFilterConfig.center_y, pcTransformFilterConfig.center_z, 
            pcTransformFilterConfig.length_x, pcTransformFilterConfig.length_y, pcTransformFilterConfig.length_z,
            pcTransformFilterConfig.rotate_x, pcTransformFilterConfig.rotate_y, pcTransformFilterConfig.rotate_z);
    
    this->TransformFilterConfig.center_x = pcTransformFilterConfig.center_x;
    this->TransformFilterConfig.center_y = pcTransformFilterConfig.center_y;
    this->TransformFilterConfig.center_z = pcTransformFilterConfig.center_z;
    this->TransformFilterConfig.length_x = pcTransformFilterConfig.length_x;
    this->TransformFilterConfig.length_y = pcTransformFilterConfig.length_y;
    this->TransformFilterConfig.length_z = pcTransformFilterConfig.length_z;
    this->TransformFilterConfig.rotate_x = pcTransformFilterConfig.rotate_x;
    this->TransformFilterConfig.rotate_y = pcTransformFilterConfig.rotate_y;
    this->TransformFilterConfig.rotate_z = pcTransformFilterConfig.rotate_z;

    this->is_updated_ = true;
}

RQTConfig::_OrthoFilterConfig_ PointcloudFilterReconfigure::getOrthoConfigure()
{
    return this->OrthoFilterCfg;
}
RQTConfig::_TransformFilterConfig_ PointcloudFilterReconfigure::getTransformConfigure()
{
    return this->TransformFilterConfig;
}

bool PointcloudFilterReconfigure::isUpdated()
{
    if(this->is_updated_)
    {
        this->is_updated_ = false;
        return true;
    }
    return false;
}