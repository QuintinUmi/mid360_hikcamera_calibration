#ifndef _RECTANGULAR_PLANE_FITTING_H_
#define _RECTANGULAR_PLANE_FITTING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // 拟合平面
#include <pcl/visualization/pcl_visualizer.h>


namespace livox_pc2_opr
{
    class RecPlaneFitting
    {
        public:
            RecPlaneFitting();

            int plane_fitting();
            int point_projection();


        private:
            std::string filePath;
            Eigen::VectorXf coefficient;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;

    };
}


#endif