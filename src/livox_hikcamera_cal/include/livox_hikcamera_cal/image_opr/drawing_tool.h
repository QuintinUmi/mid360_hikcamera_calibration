#ifndef SPACE_3D_DRAWING_H_
#define SPACE_3D_DRAWING_H_

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

#include "opencv2/opencv.hpp"   
// #include "apriltag/apriltag.h" 
#include "opencv2/aruco/charuco.hpp"    

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "param_code.h"


using namespace std;


namespace drt{

    class Draw3D{

        public:
            
            Draw3D(float unitLength = 1.0 ,float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
            Draw3D(cv::Mat cameraMatrix, cv::Mat disCoffes);

            void write_in(cv::Point3f &dst, float x, float y, float z);
            void write_in(vector<cv::Point3f> &dst, float x, float y, float z);


            void set_scale(float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);
            void set_unitlen(float unitLength);

            vector<vector<cv::Point3f>> draw_ortho_coordinate_3d(cv::Point3f centerPoint, float density = 0.1);
            vector<vector<cv::Point3f>> draw_ortho_coordinate_3d(float cx = 0.0, float cy = 0.0, float cz = 0.0, float density = 0.1);
            void draw_ortho_coordinate_2d(cv::Mat &imgInputOutput, cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                            float cx = 0.0, float cy = 0.0, float cz = 0.0);

            void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec);
            void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                            float rx, float ry, float rz, float tx, float ty, float tz);

            void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec);
            void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ);

            void setparam_image_perspective_3d(cv::Mat cameraMatrix, cv::Mat disCoffes,
                                    cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat rvec, cv::Mat tvec);
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, vector<cv::Mat> rvecs, vector<cv::Mat> tvecs);
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat cameraMatrix, cv::Mat disCoffes, 
                                            cv::Mat rvec, cv::Mat tvec, cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat cameraMatrix, cv::Mat disCoffes, 
                                            vector<cv::Mat> rvecs, vector<cv::Mat> tvecs, cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());

            void center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage);
            void center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage, float scaleX, float scaleY, int flags = 1, int borderMode = 0, const cv::Scalar &borderValue = cv::Scalar());



            // cv::Mat cal_2vec_rvec(cv::Point3f vecOri, cv::Point3f vecDst);

        private:

            float unitLength;
            float scaleX;
            float scaleY;
            float scaleZ;

            cv::Mat scaleMatrix = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0,
                                                            0.0, 1.0, 0.0,
                                                            0.0, 0.0, 1.0);

            // cv::Mat cameraMatrix;
            // cv::Mat disCoffes;


            cv::Mat setCameraMatrix;
            cv::Mat setDisCoffes;
            cv::Mat setRvec;
            cv::Mat setTvec;                             
            cv::Point3f setImgOriPoint = cv::Point3f(0.0, 0.0, 0.0);
            cv::Size setImgSizeIn3d = cv::Size(1.0, 1.0);
            cv::Mat setOffsetRvec = cv::Mat();
            cv::Mat setOffsetTvec = cv::Mat();
    };


    
}

#endif