#ifndef _IMAGE_PROCESS_H_
#define _IMAGE_PROCESS_H_

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

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/image_opr/image_process.h"

using namespace std;

namespace livox_hikcamera_cal
{

    namespace image_opr
    {

        class ImageProc
        {
            public:
                ImageProc(){}
                ~ImageProc(){}


                static void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec);
                static void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Vec3d rvec, cv::Vec3d tvec);
                static void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                                float rx, float ry, float rz, float tx, float ty, float tz);

                static void transform_3d_points_inv(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Vec3d rvec, cv::Vec3d tvec);


                static void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec);
                static void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ);

                static void estimate_average_pose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec);
                static Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions);
        };

    }
}


#endif