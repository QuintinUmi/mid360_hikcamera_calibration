#ifndef _CONVERSION_BRIDGE_H_
#define _CONVERSION_BRIDGE_H_

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



namespace livox_hikcamera_cal
{
    class ConversionBridge
    {
        public:
            ConversionBridge();
            ~ConversionBridge();

            static Eigen::Vector3f rvecCvToEigen(const cv::Mat& cv_rvec);
            static Eigen::Vector3f tvecCvToEigen(const cv::Mat& cv_tvec);
            static std::vector<Eigen::Vector3f> rvecCvToEigen(const std::vector<cv::Mat>& cv_rvecs);
            static std::vector<Eigen::Vector3f> tvecCvToEigen(const std::vector<cv::Mat>& cv_tvecs);
            static cv::Mat rvecEigenToCv(const Eigen::Vector3f& eg_rvec);
            static cv::Mat tvecEigenToCv(const Eigen::Vector3f& eg_tvec);
            static std::vector<cv::Mat> rvecEigenToCv(const std::vector<Eigen::Vector3f>& eg_rvecs);
            static std::vector<cv::Mat> tvecEigenToCv(const std::vector<Eigen::Vector3f>& eg_tvecs);

            
    };
}


using CBridge = livox_hikcamera_cal::ConversionBridge;

#endif