#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include "image_transport/image_transport.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <opencv2/core/eigen.hpp> 

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/conversion_bridge.h"

#include "livox_hikcamera_cal/image_opr/image_process.h"


using namespace livox_hikcamera_cal::image_opr;



void ImageProc::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec)
{
    if(rvec.empty()){
        rvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    if(tvec.empty()){
        tvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}
void ImageProc::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Vec3d rvec, cv::Vec3d tvec)
{
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Mat rvec_mat = (cv::Mat_<float>(3,1) << rvec[0], rvec[1], rvec[2]);
    cv::Mat tvec_mat = (cv::Mat_<float>(3,1) << tvec[0], tvec[1], tvec[2]);
    cv::Rodrigues(rvec_mat, R);
    cv::Mat tvec_(tvec);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec_mat;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}
void ImageProc::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                        float rx, float ry, float rz, float tx, float ty, float tz)
{
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << rx, ry, rz), tvec = (cv::Mat_<float>(3, 1) << tx, ty, tz);
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}

void ImageProc::transform_3d_points_inv(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Vec3d rvec, cv::Vec3d tvec)
{
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Mat rvec_mat = (cv::Mat_<float>(3,1) << rvec[0], rvec[1], rvec[2]);
    cv::Mat tvec_mat = (cv::Mat_<float>(3,1) << tvec[0], tvec[1], tvec[2]);
    cv::Rodrigues(rvec_mat, R);
    
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R.inv() * (srcMatrix - tvec_mat);
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}


void ImageProc::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec)
{
    float len = pow(surfaceNorVec.x, 2) + pow(surfaceNorVec.y, 2) + pow(surfaceNorVec.z, 2);
    float uX = surfaceNorVec.x / len;
    float uY = surfaceNorVec.y / len;
    float uZ = surfaceNorVec.z / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
            
    }
}
void ImageProc::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ)
{
    cv::Mat surfaceNorVec;
    float len = pow(surNorX, 2) + pow(surNorY, 2) + pow(surNorZ, 2);
    float uX = surNorX / len;
    float uY = surNorY / len;
    float uZ = surNorZ / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}



void ImageProc::estimate_average_pose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec)
{
    int count = 0;
    cv::Vec3d sumTvecs(0, 0, 0);
    for(const auto& tvec : tvecs) 
    {
        sumTvecs += tvec;
        count ++;
    }
    averageTvec = cv::Vec3d(sumTvecs[0] / count, sumTvecs[1] / count, sumTvecs[2] / count);


    std::vector<Eigen::Quaterniond> quaternions = ConversionBridge::rvecs3dToQuaternions(rvecs);

    Eigen::Quaterniond averageQuaternion = ImageProc::averageQuaternions(quaternions);

    averageRvec = ConversionBridge::quaternionToRvec3d(averageQuaternion);
}

Eigen::Quaterniond ImageProc::averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions) 
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (const auto& q : quaternions) {
        Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
        M += q_vec * q_vec.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_decomposition(M);
    auto eigen_vectors = eigen_decomposition.eigenvectors();
    auto eigen_values = eigen_decomposition.eigenvalues();

    int max_index = 0;
    double max_eigen_values = eigen_values(0);
    for(int i = 1; i <= 4; i++)
    {
        if(eigen_values(i) > max_eigen_values) 
        {
            max_eigen_values = eigen_values(i);
            max_index = i;
        }
    }

    Eigen::Vector4d max_eigen_vector = eigen_vectors.col(max_index);

    return Eigen::Quaterniond(max_eigen_vector(0), max_eigen_vector(1), max_eigen_vector(2), max_eigen_vector(3)).normalized();
}




