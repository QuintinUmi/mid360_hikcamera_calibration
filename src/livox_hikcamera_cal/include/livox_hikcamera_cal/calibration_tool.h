#ifndef _CALIBRATION_SOLVER_H_
#define _CALIBRATION_SOLVER_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

namespace livox_hikcamera_cal
{
    class CalTool
    {
        public:

            CalTool();
            ~CalTool();

            void sortPointByNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal);
            void sortPointByNormal(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal);

            int SolveSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output);


            static Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions);

            
        private:
        
            Eigen::Matrix3d R_;
            Eigen::Vector3d t_;
    };
}



#endif