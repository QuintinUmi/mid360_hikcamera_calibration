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
        pc_process.PassThroughFilter("z", 0, 4000);

        std::vector<cv::Point2f> imagePoints;

        d3d.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);

        d3d.drawPointsOnImageZ(*pc_process.getProcessedPointcloud(), imagePoints, img);


        cv::imshow("Projected Points", img);
        int key = cv::waitKey(1); 

        std::string command_received = command_handler.getCommand();

        if(command_received == "capture" || key == 13)
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            cornerset_csv_operator.writePointsToCSVAppend(pc_corners_rcv, img_corners_rcv);

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            cornerset_csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

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

            pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            pc_process.setCloud(pc_SUB_PUB.getPointcloudXYZI());
            pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
            pc_process.normalClusterExtraction();
            pc_process.extractNearestClusterCloud();
            pc_process.planeSegmentation();

            PointCloud2Proc caliboard_pc_proc(true);
            caliboard_pc_proc.setCloud(pc_process.getProcessedPointcloud());
            caliboard_pc_proc.transform(R, t);
            caliboard_pc_proc.scaleTo(1000.0f);
            caliboard_pc_proc.PassThroughFilter("z", 0, 4000);

            std::vector<cv::Point2f> caliboard_pc;
            auto img_caliboard_pc = img_SUB_PUB.getImage();
            d3d.projectPointsToImage(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc);
            d3d.drawPointsOnImageZ(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc, img_caliboard_pc);
            cv::imshow("caliboard_pc", img_caliboard_pc);



            pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*pc_process.calculateConcaveHull(pc_process.getProcessedPointcloud(), concave_hull_alpha), *hull_cloud);
            CalTool::removeBoundingBoxOutliers(hull_cloud, pc_corners_rcv);
            pc_process.setCloud(hull_cloud);

            // pc_process.extractConvexHull();
            pc_process.transform(R, t);
            pc_process.scaleTo(1000.0f);
            pc_process.PassThroughFilter("z", 0, 4000);

            std::vector<cv::Point2f> imagePoints;
            auto img_hull = img_SUB_PUB.getImage();
            d3d.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);
            d3d.drawPointsOnImageZ(*pc_process.getProcessedPointcloud(), imagePoints, img_hull);


            cv::imshow("concave_hull_cloud", img_hull);

            
            // double reprojection_error = CalTool::computeReprojectionErrors(hull_cloud, img_corners_raw, R, t, newCameraMatrix, newDisCoffes);
            double pixel_mean_error, pixel_stddev_error;
            CalTool::computeReprojectionErrorsInPixels(hull_cloud, img_corners_raw, R, t, newCameraMatrix, newDisCoffes, pixel_mean_error, pixel_stddev_error);
            

            // std::cout << "Reprojection Errors = " << reprojection_error << " mm" << std::endl;
            std::cout << "Reprojection Mean Errors In Pixels = " << pixel_mean_error << " pixels" << std::endl;
            std::cout << "Reprojection Standard Deviation Errors In Pixels = " << pixel_stddev_error << " pixels" << std::endl;


            command_handler.sendCommand("capture_complete");
            command_handler.resetReceivedStatus();
        }
        if(command_received == "undo" || command_received == "delete_once" || key == 8)
        {
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();

            std::vector<geometry_msgs::Point32> pc_corners_raw;
            std::vector<geometry_msgs::Point32> img_corners_raw;

            cornerset_csv_operator.readPointsFromCSV(pc_corners_raw, img_corners_raw);

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

            command_handler.sendCommand("delete_once_complete");
            command_handler.resetReceivedStatus();
        }

        if(command_received == "capture_border")
        {
            
            std::vector<geometry_msgs::Point32> img_corners_rcv = img_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> pc_corners_rcv = pc_corners_SUB_PUB.getCornersPoints32();
            pc_process.setCloud(pc_SUB_PUB.getPointcloudXYZI());
            pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
            pc_process.normalClusterExtraction();
            pc_process.extractNearestClusterCloud();
            pc_process.planeSegmentation();

            // test caliboard_pc
            PointCloud2Proc caliboard_pc_proc(true);
            caliboard_pc_proc.setCloud(pc_process.getProcessedPointcloud());
            caliboard_pc_proc.transform(R, t);
            caliboard_pc_proc.scaleTo(1000.0f);
            caliboard_pc_proc.PassThroughFilter("z", 0, 4000);

            std::vector<cv::Point2f> caliboard_pc;
            auto img_caliboard_pc = img_SUB_PUB.getImage();
            d3d.projectPointsToImage(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc);
            d3d.drawPointsOnImageZ(*caliboard_pc_proc.getProcessedPointcloud(), caliboard_pc, img_caliboard_pc);
            cv::imshow("caliboard_pc", img_caliboard_pc);
            // test caliboard_pc //


            pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*pc_process.calculateConcaveHull(pc_process.getProcessedPointcloud(), concave_hull_alpha), *hull_cloud);
            CalTool::removeBoundingBoxOutliers(hull_cloud, pc_corners_rcv);
            pc_process.setCloud(hull_cloud);

            boarderset_csv_operator.writePointsToCSVAppend(hull_cloud, img_corners_rcv);


            // pc_process.extractConvexHull();
            pc_process.transform(R, t);
            pc_process.scaleTo(1000.0f);
            pc_process.PassThroughFilter("z", 0, 4000);

            std::vector<cv::Point2f> imagePoints;
            auto img_hull = img_SUB_PUB.getImage();
            d3d.projectPointsToImage(*pc_process.getProcessedPointcloud(), imagePoints);
            d3d.drawPointsOnImageZ(*pc_process.getProcessedPointcloud(), imagePoints, img_hull);


            cv::imshow("concave_hull_cloud", img_hull);

            

            pcl::PointCloud<pcl::PointXYZI>::Ptr border_clouds(new pcl::PointCloud<pcl::PointXYZI>);
            std::vector<std::vector<geometry_msgs::Point32>> image_corner_sets;
            boarderset_csv_operator.readPointsFromCSV(border_clouds, image_corner_sets);
            // double reprojection_error = CalTool::computeReprojectionErrors(hull_cloud, img_corners_raw, R, t, newCameraMatrix, newDisCoffes);
            double pixel_mean_error, pixel_stddev_error;
            CalTool::computeReprojectionErrorsInPixels(border_clouds, image_corner_sets, R, t, newCameraMatrix, newDisCoffes, pixel_mean_error, pixel_stddev_error);
            

            // std::cout << "Reprojection Errors = " << reprojection_error << " mm" << std::endl;
            std::cout << "Reprojection Mean Errors In Pixels = " << pixel_mean_error << " pixels" << std::endl;
            std::cout << "Reprojection Standard Deviation Errors In Pixels = " << pixel_stddev_error << " pixels" << std::endl;


            command_handler.sendCommand("capture_border_complete");
            command_handler.resetReceivedStatus();
        }




        if(key == 27) break;
        
        

        rate.sleep();

    }

    return 0;
}

