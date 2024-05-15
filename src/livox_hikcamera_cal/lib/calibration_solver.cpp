#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "livox_hikcamera_cal/calibration_solver.h"

using namespace livox_hikcamera_cal;


CalSolver::CalSolver()
{

}
CalSolver::~CalSolver()
{

}



int CalSolver::SolveSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output)
{
    if (pointcloud_point_list->points.size() != image_points_list.size()) {
        return -1; 
    }


    Eigen::Vector3d centroid_pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_img = Eigen::Vector3d::Zero();
    int n = pointcloud_point_list->points.size();


    for (int i = 0; i < n; ++i) {
        centroid_pc += Eigen::Vector3d(pointcloud_point_list->points[i].x, pointcloud_point_list->points[i].y, pointcloud_point_list->points[i].z);
        centroid_img += Eigen::Vector3d(image_points_list[i].x, image_points_list[i].y, image_points_list[i].z);
    }
    centroid_pc /= n;
    centroid_img /= n;


    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d pc(pointcloud_point_list->points[i].x, pointcloud_point_list->points[i].y, pointcloud_point_list->points[i].z);
        Eigen::Vector3d img(image_points_list[i].x, image_points_list[i].y, image_points_list[i].z);
        H += (pc - centroid_pc) * (img - centroid_img).transpose();
    }


    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    if (U.determinant() * V.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }


    Eigen::Vector3d t = centroid_img - R * centroid_pc;

    R_output = R;
    t_output = t;

    return 0; 

}