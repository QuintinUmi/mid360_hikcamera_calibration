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
    class CalSolver
    {
        public:

            CalSolver();
            ~CalSolver();


        private:

            int SolveSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output);

        private:
        
            Eigen::Matrix3d R_;
            Eigen::Vector3d t_;
    };
}



#endif