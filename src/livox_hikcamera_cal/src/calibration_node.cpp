#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle rosHandle;

    

    std::string frame_id;

    std::string topic_pc_sub;
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
    fs["newCameraMatrix"] >> newCameraMatrix;
    fs["newDisCoffes"] >> newDisCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    std::string csv_path;
    std::string extrinsics_path;
    rosHandle.param("pointset_save_path", csv_path, std::string("src/livox_hikcamera_cal/data/point_set.csv"));
    rosHandle.param("extrinsics_save_path", extrinsics_path, std::string("src/livox_hikcamera_cal/cfg/extrinsics.yaml"));
    


    PointCloudSubscriberPublisher pc_SUB_PUB(rosHandle, topic_pc_sub, topic_pc_pub);

    ImageSubscriberPublisher img_SUB_PUB(rosHandle, topic_img_sub, topic_img_pub);

    CornersPublisherSubscriber pc_corners_SUB_PUB(rosHandle, frame_id, topic_pc_corners_sub, topic_corners_pub);

    CornersPublisherSubscriber img_corners_SUB_PUB(rosHandle, frame_id, topic_img_corners_sub, topic_corners_pub);


    PointCloud2Proc pc_process(true);

    Draw3D d3d(arucoRealLength[0], 1, 1, 1, cameraMatrix, disCoffes);

    CsvOperator csv_operator(csv_path);
    YamlOperator yaml_operator(extrinsics_path);

    CommandHandler command_handler(rosHandle, topic_command_sub, topic_command_pub);

    
    ros::Rate rate(30);


    pcl::PointCloud<pcl::PointXYZI> originalCloud; 
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); 
    Eigen::Vector3f t(0.0f, 0.0f, 0.0f); 

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

        pc_process.transform(R, t);
        pc_process.scaleTo(1000.0f);
        pc_process.PassThroughFilter("z", 0, 4000);

        std::vector<cv::Point2f> imagePoints;

        d3d.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);

        d3d.drawPointsOnImageZ(*pc_process.getProcessedPointcloud(), imagePoints, img);



        std::string command_received = command_handler.getCommand();

        if(command_received == "capture")
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            csv_operator.writePointsToCSVAppend(pc_corners_rcv, img_corners_rcv);

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
            }

            pc_corners_SUB_PUB.publish(pc_corners_rcv, pc_corners_SUB_PUB.nowHeader());

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("capture_complete");
            command_handler.resetReceivedStatus();
        }
        if(command_received == "undo" || command_received == "delete_once")
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
            }

            pc_corners_SUB_PUB.publish(pc_corners_rcv, pc_corners_SUB_PUB.nowHeader());

            std::cout << R << std::endl << t << std::endl;

            command_handler.sendCommand("undo_complete");
            command_handler.resetReceivedStatus();
        }


        cv::imshow("Projected Points", img);
        int key = cv::waitKey(1); 
        if (key == 27) break;   // Press 'ESC' For Exit
        if (key == 13)          // Press 'Enter' For Calibration
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            csv_operator.writePointsToCSVAppend(pc_corners_rcv, img_corners_rcv);

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
            }

            pc_corners_SUB_PUB.publish(pc_corners_rcv, pc_corners_SUB_PUB.nowHeader());

            std::cout << R << std::endl << t << std::endl;
        }
        if (key == 8)          // Press 'Enter' For Calibration
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if (!pc_corners_raw.empty()) {
                csv_operator.deleteRowFromCSV(pc_corners_raw.size()-0);
                csv_operator.deleteRowFromCSV(pc_corners_raw.size()-1);
                csv_operator.deleteRowFromCSV(pc_corners_raw.size()-2);
                csv_operator.deleteRowFromCSV(pc_corners_raw.size()-3);
            }

            csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            Eigen::Vector3f center_pc = CalTool::computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = CalTool::computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            CalTool::alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            CalTool::alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = CalTool::findRotationByICP(pc_center_refer, img_center_refer);
            t = CalTool::findTranslation(center_pc, center_img, R);

            yaml_operator.writeExtrinsicsToYaml(R, t);

            for(auto& pc_corner: pc_corners_rcv)
            {
                Eigen::Vector3f corner_trans(pc_corner.x, pc_corner.y, pc_corner.z);
                corner_trans = R * corner_trans + t;
                pc_corner.x = corner_trans.x();
                pc_corner.y = corner_trans.y();
                pc_corner.z = corner_trans.z();
            }

            pc_corners_SUB_PUB.publish(pc_corners_rcv, pc_corners_SUB_PUB.nowHeader());

            std::cout << R << std::endl << t << std::endl;
        }


        
        

        rate.sleep();

    }

    return 0;
}

