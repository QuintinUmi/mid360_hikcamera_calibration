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

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h"

#include "livox_hikcamera_cal/image_opr/image_subscriber_publisher.h"
#include "livox_hikcamera_cal/image_opr/drawing_tool.h"
#include "livox_hikcamera_cal/image_opr/aruco_tool.h"

#include "livox_hikcamera_cal/conversion_bridge.h"
#include "livox_hikcamera_cal/dynamic_reconfigure.h"
#include "livox_hikcamera_cal/rviz_drawing.h"
#include "livox_hikcamera_cal/calibration_tool.h"

#define PI 3.14159265358979324


using namespace std;
using namespace livox_hikcamera_cal;
using namespace livox_hikcamera_cal::image_opr;
using namespace livox_hikcamera_cal::pointcloud2_opr;

vector<cv::Point3f> caliboard_corners;
cv::Point3f caliboard_corner1(-76.5, 40.0, 0);
cv::Point3f caliboard_corner2(152*3+67, 40.0, 0);
cv::Point3f caliboard_corner3(152*3+67, -151.5-150, 0);
cv::Point3f caliboard_corner4(-76.5, -151.5-150, 0);
// (-76.5, -40.0, 0) (152*3+67, -40.0, 0) (152*3+67, 151.5+150, 0) (-76.5, 151.5+150, 0)

bool g_exit = false;

int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

cv::Mat testPoint;

void imgStreamReceiveCallBack(const sensor_msgs::ImageConstPtr& pImgStream, ros::NodeHandle& rosHandle, image_transport::Publisher& imgStreamPub, ArucoM& arucoMarker, Draw3D& d3d)
{
    
    struct timeval time;
    time_t startTime, endTime;

    gettimeofday(&time, NULL);
    startTime = time.tv_sec*1000 + time.tv_usec/1000;

    vector<cv::Mat> rvecs, tvecs;
    pcl::Indices detectedIds;
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(pImgStream, std::string("bgr8"));
    cv::Mat image = cvPtr->image;

    
    arucoMarker.ext_calib_multipul_arucos(image, rvecs, tvecs, detectedIds);

    

    // d3d.draw_ortho_coordinate_2d(image, d3d.getCameraMatrix(), d3d.getDisCoffes(), rvecs, tvecs);
    
    // arucoMarker.ext_calib_single_arucos(image, 544, rvecs, tvecs);

    if(!(rvecs.empty() || tvecs.empty()))
    {
        testPoint = tvecs[0];
        // d3d.paste_image_perspective_3d(stackImage, image, true, true, rvecs, tvecs);
        d3d.draw_ortho_coordinate_2d(image, rvecs, tvecs);
        // d3d.draw_line_2d(image, tvecs[0].at<float>(0, 0), tvecs[0].at<float>(1, 0), tvecs[0].at<float>(2, 0), tvecs[1].at<float>(0, 0), tvecs[1].at<float>(1, 0), tvecs[1].at<float>(2, 0), cv::Scalar(0, 255, 0), d3d.getCameraMatrix(), d3d.getDisCoffes(), rvecs[0], tvecs[0]);
        // d3d.draw_line_2d(image, caliboard_corner2, caliboard_corner3, cv::Scalar(0, 255, 0), rvecs[0], tvecs[0]);
        // d3d.draw_line_2d(image, caliboard_corner3, caliboard_corner4, cv::Scalar(0, 255, 0), rvecs[0], tvecs[0]);
        // d3d.draw_line_2d(image, caliboard_corner4, caliboard_corner1, cv::Scalar(0, 255, 0), rvecs[0], tvecs[0]);
    }
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(cvPtr->header, "bgr8", image).toImageMsg();
    imgStreamPub.publish(msg);
    
    gettimeofday(&time, NULL);
    endTime = time.tv_sec*1000 + time.tv_usec/1000;

    printf("FPS: %d\n", fps(endTime - startTime));
    // cv::imshow("AR_image_stream", image);
    // char keyInput = (char)cv::waitKey(1);
    // if(keyInput == 27)
    //     g_exit = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Aruco_Video_Ext_Calib");
    ros::NodeHandle rosHandle;

    cv::String packagePath;
    if (!rosHandle.getParam("package_path", packagePath)) {
        ROS_ERROR("Failed to get 'package_path' from the parameter server.");
        return 1;
    }
    std::cout << "package_path: " << packagePath << std::endl;
    int chdir_flags = chdir(packagePath.c_str());
    if (chdir_flags != 0) {
        perror("Error changing directory");  
        return 1;  
    }
    
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
    rosHandle.param("topic_image_stream", topicImageStream, std::string("/hikcamera/img_stream"));

    int dictionaryName;
    vector<int> ids;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});

    cv::String intrinsicsPath = yamlPath + "camera_intrinsics.yaml";
    boost::filesystem::path p = boost::filesystem::current_path();  // 获取当前路径
    std::cout << "Current working directory: " << p << std::endl;
    std::cout << intrinsicsPath << std::endl;
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

    
    Draw3D d3d(arucoRealLength[0], 1, 1, 1, newCameraMatrix, newDisCoffes);
    d3d.setparam_image_perspective_3d(newCameraMatrix, newDisCoffes, cv::Point3f(0, 0, 0), cv::Size(arucoRealLength[0] * 2, arucoRealLength[0] * 2), 
                                        (cv::Mat_<float>(3, 1) << 0, 0, -PI/2));


    caliboard_corners.emplace_back(caliboard_corner1);
    caliboard_corners.emplace_back(caliboard_corner2);
    caliboard_corners.emplace_back(caliboard_corner3);
    caliboard_corners.emplace_back(caliboard_corner4);


    RvizDrawing rviz_drawing("test", "livox_frame");

    // std::string assets_path;
    // rosHandle.param("assets_path", assets_path, std::string("~/"));
    // std::string stack_image_path = assets_path + "90885055.jpeg";
    // cv::Mat stackImage = cv::imread(stack_image_path);
    // if(stackImage.empty())
    // {
    //     ROS_ERROR("Cannot Open stackImage!\n");
    //     return 0;
    // }
    // std::cout << "stackImage.size() = " << stackImage.size() << std::endl;
    

    image_transport::ImageTransport imgIt(rosHandle);
    image_transport::Publisher imgStreamPub = imgIt.advertise("/hikcamera/img_stream_proc", 1);
    image_transport::Subscriber imgStreamSub = imgIt.subscribe(topicImageStream, 1, 
                                boost::bind(&imgStreamReceiveCallBack, _1, rosHandle, imgStreamPub, arucoMarker, d3d));
    // cv::namedWindow("AR_image_stream", cv::WINDOW_NORMAL);
    
    while(ros::ok())
    {
        ros::spinOnce();

        if(!testPoint.empty())
        {
            rviz_drawing.addPoint("point0", testPoint.at<float>(2, 0) / 1000, -testPoint.at<float>(0, 0) / 1000, -testPoint.at<float>(1, 0) / 1000, 0.3, 1.0, 0.0, 1.0);
            // std::cout << testPoint << std::endl;
        }
        else
        {
            rviz_drawing.deleteObject("point0");
        }
        rviz_drawing.publish();

        if(g_exit)
            break;
    }

    return 0;
}

