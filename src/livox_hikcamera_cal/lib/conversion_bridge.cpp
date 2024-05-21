#include <ros/ros.h>
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
#include <boost/filesystem.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <aruco/aruco.h>  
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"



#include "livox_hikcamera_cal/conversion_bridge.h"


using namespace livox_hikcamera_cal;


Eigen::Vector3f ConversionBridge::rvecCvToEigen(const cv::Mat& cv_rvec)
{
    assert(cv_rvec.rows == 3 && cv_rvec.cols == 1);
    Eigen::Vector3f eg_rvec;
    eg_rvec << cv_rvec.at<float>(0, 0), cv_rvec.at<float>(1, 0), cv_rvec.at<float>(2, 0);
    return eg_rvec;
}
Eigen::Vector3f ConversionBridge::tvecCvToEigen(const cv::Mat& cv_tvec)
{
    assert(cv_tvec.rows == 3 && cv_tvec.cols == 1);
    Eigen::Vector3f eg_tvec;
    eg_tvec << cv_tvec.at<float>(0, 0), cv_tvec.at<float>(1, 0), cv_tvec.at<float>(2, 0);
    return eg_tvec;
}
std::vector<Eigen::Vector3f> ConversionBridge::rvecCvToEigen(const std::vector<cv::Mat>& cv_rvecs) {
    std::vector<Eigen::Vector3f> eg_rvecs;
    eg_rvecs.reserve(cv_rvecs.size());
    for (const auto& rvec : cv_rvecs) {
        assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_32F);
        Eigen::Vector3f eg_rvec;
        eg_rvec << rvec.at<float>(0, 0), rvec.at<float>(1, 0), rvec.at<float>(2, 0);
        eg_rvecs.push_back(eg_rvec);
    }
    return eg_rvecs;
}
std::vector<Eigen::Vector3f> ConversionBridge::tvecCvToEigen(const std::vector<cv::Mat>& cv_tvecs) {
    std::vector<Eigen::Vector3f> eg_tvecs;
    eg_tvecs.reserve(cv_tvecs.size());
    for (const auto& tvec : cv_tvecs) {
        assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_32F);
        Eigen::Vector3f eg_tvec;
        eg_tvec << tvec.at<float>(0, 0), tvec.at<float>(1, 0), tvec.at<float>(2, 0);
        eg_tvecs.push_back(eg_tvec);
    }
    return eg_tvecs;
}

cv::Mat ConversionBridge::rvecEigenToCv(const Eigen::Vector3f& eg_rvec) {
    cv::Mat cv_rvec(3, 1, CV_32F);
    for (int i = 0; i < 3; ++i) {
        cv_rvec.at<float>(i, 0) = eg_rvec[i];
    }
    return cv_rvec;
}
cv::Mat ConversionBridge::tvecEigenToCv(const Eigen::Vector3f& eg_tvec) {
    cv::Mat cv_tvec(3, 1, CV_32F);
    for (int i = 0; i < 3; ++i) {
        cv_tvec.at<float>(i, 0) = eg_tvec[i];
    }
    return cv_tvec;
}
std::vector<cv::Mat> ConversionBridge::rvecEigenToCv(const std::vector<Eigen::Vector3f>& eg_rvecs) {
    std::vector<cv::Mat> cv_rvecs;
    cv_rvecs.reserve(eg_rvecs.size());
    for (const auto& rvec : eg_rvecs) {
        cv::Mat cv_rvec(3, 1, CV_32F);
        for (int i = 0; i < 3; ++i) {
            cv_rvec.at<float>(i, 0) = rvec[i];
        }
        cv_rvecs.push_back(cv_rvec);
    }
    return cv_rvecs;
}
std::vector<cv::Mat> ConversionBridge::tvecEigenToCv(const std::vector<Eigen::Vector3f>& eg_tvecs) {
    std::vector<cv::Mat> cv_tvecs;
    cv_tvecs.reserve(eg_tvecs.size());
    for (const auto& tvec : eg_tvecs) {
        cv::Mat cv_tvec(3, 1, CV_32F);
        for (int i = 0; i < 3; ++i) {
            cv_tvec.at<float>(i, 0) = tvec[i];
        }
        cv_tvecs.push_back(cv_tvec);
    }
    return cv_tvecs;
}

static Eigen::Vector3f cv3fToEigen3f(const cv::Vec3f& cv_vec)
{
    return Eigen::Vector3f(cv_vec[0], cv_vec[1], cv_vec[2]);
}
static cv::Vec3f eigen3fToCv3f(const Eigen::Vector3f& eg_vec)
{
    return cv::Vec3f(eg_vec.x(), eg_vec.y(), eg_vec.z());
}

static cv::Vec3f cv3dToCv3f(const cv::Vec3d& cv_vec)
{
    return cv::Vec3f(cv_vec[0], cv_vec[1], cv_vec[2]);
}
static cv::Vec3d cv3fToCv3d(const cv::Vec3f& cv_vec)
{
    return cv::Vec3d(cv_vec[0], cv_vec[1], cv_vec[2]);
}

std::vector<cv::Mat> ConversionBridge::rvecToRmat(const std::vector<cv::Vec3d>& rvec) 
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    return rmat;
}
std::vector<cv::Mat> ConversionBridge::rvecsToRmats(const std::vector<cv::Vec3d>& rvecs) 
{
    std::vector<cv::Mat> rmats;
    for (const auto& rvec : rvecs) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        rmats.push_back(R);
    }
    return rmats;
}

Eigen::Quaterniond ConversionBridge::rvec3dToQuaternion(const cv::Vec3d& rvec) 
{
    cv::Mat rotMat;
    cv::Rodrigues(rvec, rotMat);  

    Eigen::Matrix3d eigenMat;
    for(int i = 0; i < rotMat.rows; ++i) 
    {
        for(int j = 0; j < rotMat.cols; ++j) 
        {
            eigenMat(i, j) = rotMat.at<double>(i, j);
        }
    }
    return Eigen::Quaterniond(eigenMat);
}

std::vector<Eigen::Quaterniond> ConversionBridge::rvecs3dToQuaternions(const std::vector<cv::Vec3d>& rvecs) 
{
    std::vector<Eigen::Quaterniond> quaternions;
    for(const auto& rvec : rvecs) 
    {
        quaternions.push_back(ConversionBridge::rvec3dToQuaternion(rvec));
    }
    return quaternions;
}

cv::Vec3d ConversionBridge::quaternionToRvec3d(const Eigen::Quaterniond& quaternion) 
{
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    cv::Mat cvRotationMatrix(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cvRotationMatrix.at<double>(i, j) = rotationMatrix(i, j);
        }
    }

    cv::Vec3d rvec;
    cv::Rodrigues(cvRotationMatrix, rvec);

    return rvec;
}

std::vector<cv::Vec3d> ConversionBridge::quaternionsToRvecs3d(const std::vector<Eigen::Quaterniond>& quaternions) 
{
    std::vector<cv::Vec3d> rvecs;
    for(const auto& quaternion : quaternions) 
    {
        rvecs.push_back(ConversionBridge::quaternionToRvec3d(quaternion));
    }
    return rvecs;
}