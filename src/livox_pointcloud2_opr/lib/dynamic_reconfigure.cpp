#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_pointcloud2_opr/PointcloudFilterConfig.h>

#include "livox_pc2_opr/dynamic_reconfigure.h"

using namespace livox_pc2_opr;


PointcloudFilterReconfigure::PointcloudFilterReconfigure() 
{
    this->f = boost::bind(&PointcloudFilterReconfigure::FilterReconfigure_CallBack, this, _1, _2);
    this->server.setCallback(this->f);
}

void PointcloudFilterReconfigure::FilterReconfigure_CallBack(livox_pointcloud2_opr::PointcloudFilterConfig &pcFilterConfig, uint32_t level) 
{
    ROS_INFO("Filter Reconfigure: x_min=%f, x_max=%f, y_min=%f, y_max=%f, z_min=%f, z_max=%f",
            pcFilterConfig.x_min, pcFilterConfig.x_max, pcFilterConfig.y_min, pcFilterConfig.y_max, pcFilterConfig.z_min, pcFilterConfig.z_max);
    
    this->filterCfg.x_min = pcFilterConfig.x_min;
    this->filterCfg.x_max = pcFilterConfig.x_max;
    this->filterCfg.y_min = pcFilterConfig.y_min;
    this->filterCfg.y_max = pcFilterConfig.y_max;
    this->filterCfg.z_min = pcFilterConfig.z_min;
    this->filterCfg.z_max = pcFilterConfig.z_max;
}

RQTConfig::FilterConfig_ PointcloudFilterReconfigure::get_configure()
{
    return this->filterCfg;
}