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

std::vector<cv::Mat> rvecToRmat(const std::vector<cv::Vec3d>& rvec) {
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    return rmat;
}
std::vector<cv::Mat> rvecsToRmats(const std::vector<cv::Vec3d>& rvecs) {
    std::vector<cv::Mat> rmats;
    for (const auto& rvec : rvecs) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        rmats.push_back(R);
    }
    return rmats;
}