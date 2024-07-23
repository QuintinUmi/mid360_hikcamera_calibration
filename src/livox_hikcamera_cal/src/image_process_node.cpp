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

#include "livox_hikcamera_cal/corners_subscriber_publisher.h"
#include "livox_hikcamera_cal/calibration_tool.h"

#define PI 3.14159265358979324


using namespace std;
using namespace livox_hikcamera_cal;
using namespace livox_hikcamera_cal::image_opr;

int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_process_node");
    ros::NodeHandle rosHandle;



    std::string frame_id;
    std::string topic_img_sub;
    std::string topic_img_pub;
	std::string topic_corners_sub;
    std::string topic_corners_pub;
    rosHandle.param("frame_id", frame_id, std::string("livox_frame"));
	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/img_stream"));
    rosHandle.param("image_process_img_pub_topic", topic_img_pub, std::string("/hikcamera/img_stream_proc"));
	rosHandle.param("image_process_corners_sub_topic", topic_corners_sub, std::string("/livox_hikcamera_cal/calibration_corners"));
    rosHandle.param("image_process_corners_pub_topic", topic_corners_pub, std::string("/livox_hikcamera_cal/image_corners"));



    std::string packagePath;
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

    cv::Size chessboardSize;
    float squareSize;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_save_path", imageSavePath, cv::String("~/"));
    rosHandle.param("image_load_path", imageLoadPath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, cv::String("png"));

    int dictionaryName;
    vector<int> ids;
    bool use_center_id;
    int center_id;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("use_center_id", use_center_id, false);
    rosHandle.param("center_id", center_id, 12);
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});

    float caliboard_width;
    float caliboard_height;

    rosHandle.param("caliboard_width", caliboard_width, (float)12.0);
    rosHandle.param("caliboard_height", caliboard_height, (float)12.0);

    cv::String intrinsicsPath = yamlPath + "camera_intrinsics.yaml";
    boost::filesystem::path p = boost::filesystem::current_path();  
    std::cout << "Current working directory: " << p << std::endl;
    std::cout << intrinsicsPath << std::endl;
    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    cv::Mat cameraMatrix, disCoffes, distortedCameraMatrix, distortedDisCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs["newCameraMatrixAlpha0"] >> distortedCameraMatrix;
    fs["newDisCoffesAlpha0"] >> distortedDisCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << distortedCameraMatrix << std::endl;
    std::cout << distortedDisCoffes << std::endl;
    std::cout << image_size << std::endl;

    cv::aruco::DICT_6X6_1000;
    ArucoM arucoMarker(dictionaryName, ids, arucoRealLength, distortedCameraMatrix, distortedDisCoffes);
    arucoMarker.setDetectionParameters();
    arucoMarker.create();


    Draw3D d3d(arucoRealLength[0], 1, 1, 1, distortedCameraMatrix, distortedDisCoffes);

    RvizDrawing rviz_drawing("/rviz_drawing/image_process_node", frame_id);
    
    ImageSubscriberPublisher img_SUB_PUB(rosHandle, topic_img_sub, topic_img_pub, "compressed");

    CornersPublisherSubscriber corners_SUB_PUB(rosHandle, frame_id, topic_corners_sub, topic_corners_pub);
    

    ros::Rate rate(50);


    while(ros::ok())
    {
        ros::spinOnce();

        auto img = img_SUB_PUB.getImage();
        

        if(img.empty())
        {
            // ROS_INFO("Waiting For Image Subscribe\n");
            continue;
        }

        std::vector<cv::Vec3d> rvecs; 
        std::vector<cv::Vec3d> tvecs;
        pcl::Indices detectedIds;
        arucoMarker.ext_calib_multipul_arucos(img, rvecs, tvecs, detectedIds);
        if(detectedIds.size() == 0)
        {
            ROS_INFO("Not Marker Detected \n");
            continue;
        }

        cv::Vec3d rvec;
        cv::Vec3d tvec;
        ImageProc::estimate_average_pose(rvecs, tvecs, rvec, tvec);


        std::vector<cv::Point3f> corners_plane;
        std::vector<cv::Point3f> corners_3d;
        cv::Point3f center;
        if(use_center_id)
        {
            
            int center_index = -1;
            for(int i = 0; i < detectedIds.size(); i ++)
            {
                if(detectedIds[i] == center_id)
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
            
            center.x = tvecs[center_index][0];
            center.y = tvecs[center_index][1];
            center.z = tvecs[center_index][2];

            
            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

            ImageProc::transform_3d_points(corners_plane, corners_3d, rvec, tvec);

            d3d.draw_ortho_coordinate_2d(img, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
            d3d.draw_line_2d(img, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvecs[center_index]), cv::Scalar(0, 0, 255));

        }
        else
        {
            
            center.x = tvec[0];
            center.y = tvec[1];
            center.z = tvec[2];

            
            corners_plane.emplace_back(-caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, +caliboard_height/2, 0);
            corners_plane.emplace_back(+caliboard_width/2, -caliboard_height/2, 0);
            corners_plane.emplace_back(-caliboard_width/2, -caliboard_height/2, 0);

            ImageProc::transform_3d_points(corners_plane, corners_3d, rvec, tvec);

            d3d.draw_ortho_coordinate_2d(img, ConversionBridge::rvecs3dToMat_d(rvecs), ConversionBridge::rvecs3dToMat_d(tvecs));
            d3d.draw_line_2d(img, corners_plane[0], corners_plane[1], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[1], corners_plane[2], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[2], corners_plane[3], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
            d3d.draw_line_2d(img, corners_plane[3], corners_plane[0], cv::Mat(rvec), cv::Mat(tvec), cv::Scalar(0, 0, 255));
        
        }

        

        

        rviz_drawing.addLine("line1", corners_3d[0].x /1000, corners_3d[0].y /1000, corners_3d[0].z /1000, corners_3d[1].x /1000, corners_3d[1].y /1000, corners_3d[1].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line2", corners_3d[1].x /1000, corners_3d[1].y /1000, corners_3d[1].z /1000, corners_3d[2].x /1000, corners_3d[2].y /1000, corners_3d[2].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line3", corners_3d[2].x /1000, corners_3d[2].y /1000, corners_3d[2].z /1000, corners_3d[3].x /1000, corners_3d[3].y /1000, corners_3d[3].z /1000, 0.01, 1.0, 0.0, 0.0);
        rviz_drawing.addLine("line4", corners_3d[3].x /1000, corners_3d[3].y /1000, corners_3d[3].z /1000, corners_3d[0].x /1000, corners_3d[0].y /1000, corners_3d[0].z /1000, 0.01, 1.0, 0.0, 0.0);
        

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        cv::Mat z_axis_vector = rotation_matrix.col(2);
        Eigen::Vector3f plane_normal;
        plane_normal << -z_axis_vector.at<double>(0), -z_axis_vector.at<double>(1), -z_axis_vector.at<double>(2);

        CalTool::sortPointByNormalImgFrame(corners_3d, plane_normal);

        std::vector<geometry_msgs::Point> ros_corners;
		for (const auto& corner : corners_3d) 
		{
			geometry_msgs::Point ros_point;
			ros_point.x = corner.x / 1000;
			ros_point.y = corner.y / 1000;
			ros_point.z = corner.z / 1000;
			ros_corners.push_back(ros_point);
    	}

        corners_SUB_PUB.publish(ros_corners, corners_SUB_PUB.nowHeader());

        rviz_drawing.addText("corner_1", ros_corners.at(0), "1", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_2", ros_corners.at(1), "2", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_3", ros_corners.at(2), "3", 0.3, 1.0, 0.0, 0.0);
        rviz_drawing.addText("corner_4", ros_corners.at(3), "4", 0.3, 1.0, 0.0, 0.0);

        Eigen::Vector3f plane_normal_(plane_normal.x(), plane_normal.y(), plane_normal.z());
        plane_normal_ = plane_normal_.normalized();

        Eigen::Vector3f center_point(0.0, 0.0, 0.0);
        for(const auto& corner:ros_corners)
        {
            center_point[0] += corner.x;
            center_point[1] += corner.y;
            center_point[2] += corner.z;
        }
        center_point /= ros_corners.size();

        float dotProduct = plane_normal_.dot(center_point);

        if (dotProduct > 0) 
        {
            plane_normal_ = -plane_normal_;
        }
        rviz_drawing.addArrow("plane_normals", 
                            center_point.x(),
                            center_point.y(),
                            center_point.z(),
                            plane_normal_, 0.03, 0.06, 0.06, 1.0, 0.0, 0.0);

        rviz_drawing.publish();

        img_SUB_PUB.publish(img);


        

        rate.sleep();

    }

    return 0;
}

