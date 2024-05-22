#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <aruco/aruco.h>  
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "livox_hikcamera_cal/image_opr/image_subscriber_publisher.h"
#include "livox_hikcamera_cal/image_opr/image_process.h"
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
    float squareSize;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_save_path", imageSavePath, cv::String("~/"));
    rosHandle.param("image_load_path", imageLoadPath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, cv::String("png"));
    rosHandle.param("topic_image_stream", topicImageStream, std::string("/hikcamera/img_stream"));

    int dictionaryName;
    vector<int> ids;
    int centered_id;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("centered_id", centered_id, 12);
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});

    float caliboard_width;
    float caliboard_hight;

    rosHandle.param("caliboard_width", caliboard_width, (float)12.0);
    rosHandle.param("caliboard_hight", caliboard_hight, (float)12.0);

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
    arucoMarker.setDetectionParameters();
    arucoMarker.create();

    ImageProc img_process;

    Draw3D d3d(arucoRealLength[0], 1, 1, 1, cameraMatrix, disCoffes);

    RvizDrawing rviz_drawing("image_process_node", "livox_frame");
    
    ImageSubscriberPublisher img_SUB_PUB;
    
    ros::Rate rate(30);


    while(ros::ok())
    {
        ros::spinOnce();

        auto img = img_SUB_PUB.getImage();

        if(img.empty())
        {
            ROS_INFO("Waiting For Image Subscribe\n");
            continue;
        }

        std::vector<cv::Vec3d> rvecs; 
        std::vector<cv::Vec3d> tvecs;
        pcl::Indices detectedIds;
        arucoMarker.ext_calib_multipul_arucos(img, rvecs, tvecs, detectedIds);
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        ImageProc::estimate_average_pose(rvecs, tvecs, rvec, tvec);

        int center_index = -1;
        for(int i = 0; i < detectedIds.size(); i ++)
        {
            if(detectedIds[i] == centered_id)
            {
                center_index = i;
            }
        }
        if(center_index == -1)
        {
            rviz_drawing.deleteObject("line1");
            rviz_drawing.deleteObject("line2");
            rviz_drawing.deleteObject("line3");
            rviz_drawing.deleteObject("line4");
            rviz_drawing.publish();
            img_SUB_PUB.publish(img);
            ROS_INFO("Not Detected Center Marker!\n");
            continue;
        }
        
        cv::Point3f center(tvecs[center_index][0], tvecs[center_index][1], tvecs[center_index][2]);

        std::vector<cv::Point3f> corners_plane;
        std::vector<cv::Point3f> corners_3d;
        corners_plane.emplace_back(-caliboard_width/2, +caliboard_hight/2, 0);
        corners_plane.emplace_back(+caliboard_width/2, +caliboard_hight/2, 0);
        corners_plane.emplace_back(+caliboard_width/2, -caliboard_hight/2, 0);
        corners_plane.emplace_back(-caliboard_width/2, -caliboard_hight/2, 0);

        std::cout << caliboard_width << " " << caliboard_hight << std::endl;

        ImageProc::transform_3d_points(corners_plane, corners_3d, rvec, tvecs[center_index]);
        // std::cout << corners_3d << std::endl;

        d3d.draw_ortho_coordinate_2d(img, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
        d3d.draw_line_2d(img, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
        d3d.draw_line_2d(img, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
        d3d.draw_line_2d(img, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
        d3d.draw_line_2d(img, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));

        rviz_drawing.addLine("line1", corners_3d[0].z /1000, -corners_3d[0].x /1000, -corners_3d[0].y /1000, corners_3d[1].z /1000, -corners_3d[1].x /1000, -corners_3d[1].y /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line2", corners_3d[1].z /1000, -corners_3d[1].x /1000, -corners_3d[1].y /1000, corners_3d[2].z /1000, -corners_3d[2].x /1000, -corners_3d[2].y /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line3", corners_3d[2].z /1000, -corners_3d[2].x /1000, -corners_3d[2].y /1000, corners_3d[3].z /1000, -corners_3d[3].x /1000, -corners_3d[3].y /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line4", corners_3d[3].z /1000, -corners_3d[3].x /1000, -corners_3d[3].y /1000, corners_3d[0].z /1000, -corners_3d[0].x /1000, -corners_3d[0].y /1000, 0.01, 1.0, 0.0, 0.0);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        cv::Mat z_axis_vector = rotation_matrix.col(2);
        Eigen::Vector3f plane_normal;
        plane_normal << -z_axis_vector.at<double>(0), -z_axis_vector.at<double>(1), -z_axis_vector.at<double>(2);

        CalTool::sortPointByNormal(corners_3d, plane_normal);

        std::vector<geometry_msgs::Point> ros_corners;
		for (const auto& corner : corners_3d) 
		{
			geometry_msgs::Point ros_point;
			ros_point.x = corner.z / 1000;
			ros_point.y = -corner.x / 1000;
			ros_point.z = -corner.y / 1000;
			ros_corners.push_back(ros_point);
    	}

        rviz_drawing.addText("corner_1", ros_corners.at(0), "1", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_2", ros_corners.at(1), "2", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_3", ros_corners.at(2), "3", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_4", ros_corners.at(3), "4", 0.3, 1.0, 0.0, 0.0);

        rviz_drawing.publish();

        img_SUB_PUB.publish(img);


        

        rate.sleep();

    }

    return 0;
}

