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

#include "livox_hikcamera_cal/corners_subscriber_publisher.h"
#include "livox_hikcamera_cal/calibration_tool.h"

#define PI 3.14159265358979324


using namespace std;
using namespace livox_hikcamera_cal;
using namespace livox_hikcamera_cal::image_opr;
using namespace livox_hikcamera_cal::pointcloud2_opr;

// vector<cv::Point3f> caliboard_corners;
// cv::Point3f caliboard_corner1(-76.5, 40.0, 0);
// cv::Point3f caliboard_corner2(152*3+67, 40.0, 0);
// cv::Point3f caliboard_corner3(152*3+67, -151.5-150, 0);
// cv::Point3f caliboard_corner4(-76.5, -151.5-150, 0);
// (-76.5, -40.0, 0) (152*3+67, -40.0, 0) (152*3+67, 151.5+150, 0) (-76.5, 151.5+150, 0)

bool g_exit = false;

int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

cv::Mat testPoint;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
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
    float squareSize;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_save_path", imageSavePath, cv::String("~/"));
    rosHandle.param("image_load_path", imageLoadPath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, cv::String("png"));
    rosHandle.param("topic_image_stream", topicImageStream, std::string("/hikcamera/img_stream"));

    int dictionaryName;
    vector<int> ids;
    int centered_ids;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("centered_id", centered_ids, 12);
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});

    cv::String intrinsicsPath = yamlPath + "camera_intrinsics.yaml";
    boost::filesystem::path p = boost::filesystem::current_path();  
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

    cv::aruco::DICT_6X6_1000;
    ArucoM arucoMarker(dictionaryName, ids, arucoRealLength, cameraMatrix, disCoffes);
    arucoMarker.create();

    Draw3D d3d(arucoRealLength[0], 1, 1, 1, cameraMatrix, disCoffes);

    RvizDrawing rviz_drawing("lidar_camera_calibration", "livox_frame");

    PointCloud2Proc pc_process;
    
    PointCloudSubscriberPublisher pc_SUB_PUB;
    
    ros::Rate rate(30);


    while(ros::ok())
    {
        ros::spinOnce();

        pc_process.setCloud(pc_SUB_PUB.getPointcloudXYZI());

        if(pc_SUB_PUB.getPointcloudXYZI()->size() == 0 || !pc_SUB_PUB.getPointcloudXYZI())
		{
			ROS_INFO("Waiting For Point Cloud Subscribe\n");
			continue;
		}

        // pcl::PointCloud<pcl::PointXYZI>::Ptr pc_corners = pc_process.extractNearestRectangleCorners(false);



        

        rate.sleep();

    }

    return 0;
}

