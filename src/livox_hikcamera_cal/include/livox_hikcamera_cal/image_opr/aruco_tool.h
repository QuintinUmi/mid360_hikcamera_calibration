#ifndef _ARUCO_TOOL_H_
#define _ARUCO_TOOL_H_

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
#include "opencv2/opencv.hpp"  
// #include "apriltag/apriltag.h"    
#include "opencv2/aruco/charuco.hpp"  
  
#include <yaml-cpp/yaml.h>

using namespace std;


namespace livox_hikcamera_cal
{

    namespace image_opr
    {
        #define SUCCESS_PROCESS true
        #define FAILD_PROCESS false

        class ArucoM{

            public:

                ArucoM();
                ArucoM(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
                ArucoM(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
                ~ArucoM();

                void create();
                void aruco_map_init();
                // void release();

                float getMarkerRealLength(int markerId);
                vector<int> getSelectedIds();
                int getSelectedIds(int index);

                    
                void set_aruco(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0});
                void set_aruco(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0});
                void sel_aruco_ids(vector<int> selectedIds);
                void set_aruco_real_length(vector<float> markerRealLength);
                void set_camera_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes);

                vector<cv::Mat> generate_aruco_marker(int markerSize);
                vector<cv::Mat> generate_aruco_marker(int dictionaryName, vector<int> selectedIds, int markerSize);
                void generate_aruco_inner(int markerSize);
                void generate_aruco_inner(int dictionaryName, vector<int> selectedIds, int markerSize);

                void detect_aruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, vector<int> markerIds);

                void ext_calib_single_arucos(cv::Mat &inputImage, int targetId, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs);
                void ext_calib_single_arucos(cv::Mat &inputImage, int targetId, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs);
                
                void ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int>& detectedIds);
                void ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs, vector<int>& detectedIds);

                void estimate_average_pose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, vector<int>& detectedIds, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec);

                void aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, int dictionaryName, bool showImage);

            private:
                
                std::map<int, int> aruco_map;
                vector<int> targetIds; 

                cv::Ptr<cv::aruco::Dictionary> markerDictionary;
                vector<int> selectedIds;
                cv::Ptr<cv::aruco::DetectorParameters> dParameters;
                vector<cv::Mat> markerImage;

                vector<float> markerRealLength;

                cv::Mat cameraMatrix;
                cv::Mat disCoffes;

        };
    }
}




#endif