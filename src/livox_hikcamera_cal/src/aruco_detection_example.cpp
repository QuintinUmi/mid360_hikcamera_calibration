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
#include <cv_bridge/cv_bridge.h>
// #include "apriltag/apriltag.h"    
#include "opencv2/aruco/charuco.hpp"  


#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "livox_hikcamera_cal/image_opr/drawing_tool.h"
#include "livox_hikcamera_cal/image_opr/aruco_tool.h"


#define PI 3.14159265358979324


using namespace std;
using namespace livox_hikcamera_cal::image_opr;


bool g_exit = false;


int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

void imgStreamReceiveCallBack(const sensor_msgs::ImageConstPtr& pImgStream, ros::NodeHandle& rosHandle, ArucoM& arucoMarker, Draw3D& d3d,cv::Mat& stackImage)
{
    
    struct timeval time;
    time_t startTime, endTime;

    gettimeofday(&time, NULL);
    startTime = time.tv_sec*1000 + time.tv_usec/1000;


    vector<cv::Mat> rvecs, tvecs;
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(pImgStream, std::string("bgr8"));
    cv::Mat image = cvPtr->image;
    arucoMarker.ext_calib_single_arucos(image, 150, rvecs, tvecs);
    if(!(rvecs.empty() || tvecs.empty()))
    {
        d3d.paste_image_perspective_3d(stackImage, image, true, true, rvecs, tvecs);
    }

    gettimeofday(&time, NULL);
    endTime = time.tv_sec*1000 + time.tv_usec/1000;

    printf("FPS: %d\n", fps(endTime - startTime));
    cv::imshow("AR_image_stream", image);
    char keyInput = (char)cv::waitKey(1);
    if(keyInput == 27)
        g_exit = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Aruco_Video_Ext_Calib");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    std::string imageFormat;
    cv::String imageSavePath;
    cv::String imageLoadPath;
    std::string topicImageStream;

    cv::Size chessboardSize;
    double squareSize;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_save_path", imageSavePath, cv::String("~/"));
    rosHandle.param("image_load_path", imageLoadPath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, cv::String("png"));
    rosHandle.param("topic_image_stream", topicImageStream, std::string("/msg_camera/img_stream"));

    int dictionaryName;
    vector<int> ids;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});
    

    cv::String intrinsicsPath = yamlPath + "calibration_param.yaml";
    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    cv::Mat cameraMatrix, disCoffes, newCameraMatrix, newDisCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs["newCameraMatrix"] >> newCameraMatrix;
    fs["newDisCoffes"] >> newDisCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    cv::aruco::DICT_6X6_250;
    ArucoM arucoMarker(dictionaryName, ids, arucoRealLength, newCameraMatrix, newDisCoffes);
    arucoMarker.create();

    
    Draw3D d3d(47.62, 1, 1, 1, newCameraMatrix, newDisCoffes);
    d3d.setparam_image_perspective_3d(newCameraMatrix, newDisCoffes, cv::Point3f(0, 0, 0), cv::Size(47.62 * 2, 47.62 * 2), 
                                        (cv::Mat_<float>(3, 1) << 0, 0, -PI/2));


    cv::Mat stackImage = cv::imread("/home/quintinumi/test.jpeg");

    
    image_transport::ImageTransport imgIt(rosHandle);
    image_transport::Subscriber imgStreamSub = imgIt.subscribe(topicImageStream, 1, 
                                                            boost::bind(&imgStreamReceiveCallBack, _1, rosHandle, arucoMarker, d3d, stackImage));

    cv::namedWindow("AR_image_stream", cv::WINDOW_NORMAL);
    
    while(ros::ok())
    {
        ros::spinOnce();
        if(g_exit)
            break;
    }

    arucoMarker.release();

    return 0;
}