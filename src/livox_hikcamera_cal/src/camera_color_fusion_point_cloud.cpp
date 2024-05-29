#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <boost/filesystem.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <cassert>

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

#include "livox_hikcamera_cal/file_operator.h"

#include "livox_hikcamera_cal/CommandHandler.h"

#define PI 3.14159265358979324


using namespace std;
using namespace livox_hikcamera_cal;
using namespace livox_hikcamera_cal::image_opr;
using namespace livox_hikcamera_cal::pointcloud2_opr;


int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}



void downscalePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud) {

    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Zero(); 
    scaleMatrix(0,0) = 1 / 1000.0f;     
    scaleMatrix(1,1) = 1 / 1000.0f;
    scaleMatrix(2,2) = 1 / 1000.0f;
    scaleMatrix(3,3) = 1;   

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*inputCloud, *write_in_cloud);
    pcl::transformPointCloud(*write_in_cloud, *inputCloud, scaleMatrix);
}

void enhanceImage(cv::Mat& image) {
    cv::Mat ycrcb;
    cv::cvtColor(image, ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> channels;
    cv::split(ycrcb, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, ycrcb);
    cv::cvtColor(ycrcb, image, cv::COLOR_YCrCb2BGR);
}

void colorPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                     const std::vector<cv::Point2f>& projectedPoints,
                     cv::Mat& image,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud) {


    enhanceImage(image);

    coloredCloud->width = inputCloud->width;
    coloredCloud->height = inputCloud->height;
    coloredCloud->is_dense = inputCloud->is_dense;
    coloredCloud->points.resize(inputCloud->points.size());  

    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        auto& pt = inputCloud->points[i];
        auto& proj = projectedPoints[i];

        if (proj.x >= 0 && proj.x < image.cols && proj.y >= 0 && proj.y < image.rows) {
            pcl::PointXYZRGB& coloredPoint = coloredCloud->points[i];
            coloredPoint.x = pt.x;
            coloredPoint.y = pt.y;
            coloredPoint.z = pt.z;

            cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(static_cast<int>(proj.x), static_cast<int>(proj.y)));
            coloredPoint.b = color[0];   // Blue channel
            coloredPoint.g = color[1];   // Green channel
            coloredPoint.r = color[2];   // Red channel
        }
    }

    downscalePointCloud(coloredCloud); 
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle rosHandle;

    

    std::string frame_id;

    std::string topic_pc_sub;
    std::string topic_pc_proc_sub;
    std::string topic_pc_pub;

    std::string topic_img_sub;
    std::string topic_img_pub;

	std::string topic_pc_corners_sub;
    std::string topic_img_corners_sub;
    std::string topic_corners_pub;

    std::string topic_command_sub;
    std::string topic_command_pub;

    rosHandle.param("frame_id", frame_id, std::string("livox_frame"));

	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/livox/lidar"));
    rosHandle.param("pointcloud_process_pc_pub_topic", topic_pc_proc_sub, std::string("/livox/lidar_proc"));
    rosHandle.param("calibration_pc_pub_topic", topic_pc_pub, std::string("/livox_hikcamera_cal/pointcloud"));

	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/img_stream"));
    rosHandle.param("calibration_img_pub_topic", topic_img_pub, std::string("/livox_hikcamera_cal/image"));

    rosHandle.param("pointcloud_process_corners_pub_topic", topic_pc_corners_sub, std::string("/livox_hikcamera_cal/pointcloud_corners"));
    rosHandle.param("image_process_corners_pub_topic", topic_img_corners_sub, std::string("/livox_hikcamera_cal/image_corners"));
    rosHandle.param("calibration_corners_pub_topic", topic_corners_pub, std::string("/livox_hikcamera_cal/calibration_corners"));

    rosHandle.param("calibration_command_sub_topic", topic_command_sub, std::string("/livox_hikcamera_cal/command_controller"));
    rosHandle.param("calibration_command_pub_topic", topic_command_pub, std::string("/livox_hikcamera_cal/command_cal_node"));



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
    rosHandle.param("topic_image_stream", topic_img_sub, std::string("/hikcamera/img_stream"));
    // rosHandle.param("topic_image_stream", topicImageStream, std::string("/hikcamera/img_stream_proc"));

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
    fs["newCameraMatrixAlpha0"] >> newCameraMatrix;
    fs["newDisCoffesAlpha0"] >> newDisCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    std::string cornerset_csv_path;
    std::string error_anaylysis_csv_path;
    std::string extrinsics_path;
    rosHandle.param("pointset_save_path", cornerset_csv_path, std::string("src/livox_hikcamera_cal/data/point_set.csv"));
    rosHandle.param("error_analysis_save_path", error_anaylysis_csv_path, std::string("src/livox_hikcamera_cal/data/border_error_anaylysis.csv"));
    rosHandle.param("extrinsics_save_path", extrinsics_path, std::string("src/livox_hikcamera_cal/cfg/extrinsics.yaml"));
    


    PointCloudSubscriberPublisher pc_SUB_PUB(rosHandle, topic_pc_sub, topic_pc_pub);
    PointCloudSubscriberPublisher pc_proc_SUB_PUB(rosHandle, topic_pc_proc_sub, topic_pc_pub);

    ImageSubscriberPublisher img_SUB_PUB(rosHandle, topic_img_sub, topic_img_pub);

    CornersPublisherSubscriber pc_corners_SUB_PUB(rosHandle, frame_id, topic_pc_corners_sub, topic_corners_pub);

    CornersPublisherSubscriber img_corners_SUB_PUB(rosHandle, frame_id, topic_img_corners_sub, topic_corners_pub);


    PointCloud2Proc pc_process(true);

    Draw3D d3d(arucoRealLength[0], 1, 1, 1, cameraMatrix, disCoffes);

    RQTConfig rqtCfg;
    // PointcloudFilterReconfigure filterRecfg(rosHandle);
    CalibrationParamReconfigure calParamRecfg(rosHandle);

    CornerSetCsvOperator cornerset_csv_operator(cornerset_csv_path);
    BorderSetCsvOperator boarderset_csv_operator(error_anaylysis_csv_path);
    // boarderset_csv_operator.writePointsToCSVOverwrite(pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>), std::vector<std::vector<geometry_msgs::Point32>>());

    YamlOperator yaml_operator(extrinsics_path);

    CommandHandler command_handler(rosHandle, topic_command_sub, topic_command_pub);

    
    ros::Rate rate(30);


    pcl::PointCloud<pcl::PointXYZI> originalCloud; 
    
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
    Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 

    yaml_operator.readExtrinsicsFromYaml(R, t);

    ros::Publisher cloud_pub = rosHandle.advertise<sensor_msgs::PointCloud2>("/livox_hikcamera_cal/color_cloud", 10);

    while(ros::ok())
    {
        ros::spinOnce();

        pc_process.setCloud(pc_SUB_PUB.getPointcloudXYZI());

        if(pc_SUB_PUB.getPointcloudXYZI()->size() == 0 || !pc_SUB_PUB.getPointcloudXYZI())
		{
			// ROS_INFO("Waiting For Point Cloud Subscribe\n");
			continue;
		}

        auto img = img_SUB_PUB.getImage();

        if(img.empty())
        {
            // ROS_INFO("Waiting For Image Subscribe\n");
            continue;
        }

        cv::Mat imgEnhanced;
        cv::convertScaleAbs(img, imgEnhanced, 1.2, 10); 
        img = imgEnhanced.clone();


		float center_x;
		float center_y;
		float center_z;
		float length_x;
		float length_y;
		float length_z;
		float rotate_x;
		float rotate_y;
		float rotate_z;

        rosHandle.getParam("/shared_parameter/center_x", center_x);
		rosHandle.getParam("/shared_parameter/center_y", center_y);
		rosHandle.getParam("/shared_parameter/center_z", center_z);
		rosHandle.getParam("/shared_parameter/length_x", length_x);
		rosHandle.getParam("/shared_parameter/length_y", length_y);
		rosHandle.getParam("/shared_parameter/length_z", length_z);
		rosHandle.getParam("/shared_parameter/rotate_x", rotate_x);
		rosHandle.getParam("/shared_parameter/rotate_y", rotate_y);
		rosHandle.getParam("/shared_parameter/rotate_z", rotate_z);

        rqtCfg.CalibrationParam = calParamRecfg.getCalibrationParamConfigure();
        float concave_hull_alpha = rqtCfg.CalibrationParam.concave_hull_alpha;
		



        // pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
        // pc_process.normalClusterExtraction();
        // pc_process.extractNearestClusterCloud();
        // pc_process.planeSegmentation();
        pc_process.transform(R, t);
        pc_process.scaleTo(1000.0f);
        pc_process.PassThroughFilter("z", 100, 40000);

        std::vector<cv::Point2f> imagePoints;

        d3d.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);

        

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colorPointCloud(pc_process.getProcessedPointcloud(), imagePoints, img, rgb_cloud);


        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.linear() = R;
        transform.translation() = t;

        Eigen::Matrix4f transform_matrix_inv = transform.matrix().inverse();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*rgb_cloud, *write_in_cloud);
        pcl::transformPointCloud(*write_in_cloud, *rgb_cloud, transform_matrix_inv);



        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*rgb_cloud, output);
        output.header.frame_id = "livox_frame";  
        // output.header.stamp = ros::Time::now();
        output.width = rgb_cloud->points.size(); 
        output.height = 1; 
        
        for (auto& point : rgb_cloud->points) {
            // ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                std::cout << "Found invalid point" << std::endl;
            }
        }
        
        cloud_pub.publish(output);

        
        d3d.drawPointsOnImageIntensity(*pc_process.getProcessedPointcloud(), imagePoints, img);


        cv::imshow("Projected Points", img);
        int key = cv::waitKey(1); 



        if(key == 27) break;
        
        

        rate.sleep();

    }

    return 0;
}

