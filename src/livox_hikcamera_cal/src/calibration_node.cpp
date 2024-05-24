#include <ros/ros.h>
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


int fps(int deltaTime) 
{
    int fps = static_cast<int>(1.f / deltaTime * 1000); 
    return fps;
}


void TransformCorners(std::vector<geometry_msgs::Point32>& points)
{
    float scale = 1000.0f;

    for(auto& point:points)
    {
        geometry_msgs::Point32 transPoint;
        transPoint.x = -point.y * scale;
        transPoint.y = -point.z * scale;
        transPoint.z = point.x * scale;
        point = transPoint;
    }
}

Eigen::Vector3f computeCentroid(const std::vector<geometry_msgs::Point32>& points) {
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& p : points) {
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= points.size();
    return centroid;
}
void alignPointsToCentroid(const std::vector<geometry_msgs::Point32>& points, const Eigen::Vector3f& centroid, Eigen::MatrixXf& out) {
    out.resize(3, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        out.col(i) = Eigen::Vector3f(points[i].x, points[i].y, points[i].z) - centroid;
    }
}
Eigen::Matrix3f findRotation(const Eigen::MatrixXf& P, const Eigen::MatrixXf& Q) {
    Eigen::Matrix3f H = P * Q.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // Check and correct for reflection
    if (U.determinant() * V.determinant() < 0) {
        U.col(2) *= -1;  // V.col(2) *= -1
    }

    return V * U.transpose();
}
Eigen::Vector3f findTranslation(const Eigen::Vector3f& centroidP, const Eigen::Vector3f& centroidQ, const Eigen::Matrix3f& R) {
    return centroidQ - R * centroidP;
}



void transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>& input,
                         pcl::PointCloud<pcl::PointXYZI>& output,
                         const Eigen::Matrix3f& R,
                         const Eigen::Vector3f& t) {
    output = input; // 复制原始点云结构
    for (size_t i = 0; i < input.points.size(); i++) {
        Eigen::Vector3f p(input.points[i].x, input.points[i].y, input.points[i].z);
        p = R * p + t; // 应用旋转和平移
        output.points[i].x = p.x();
        output.points[i].y = p.y();
        output.points[i].z = p.z();
    }
}

void projectToImage(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                    const cv::Mat& cameraMatrix,
                    const cv::Mat& distCoeffs,
                    std::vector<cv::Point2f>& imagePoints) {
    std::vector<cv::Point3f> cvPoints;
    for (const auto& p : cloud.points) {
        cvPoints.push_back(cv::Point3f(p.x, p.y, p.z));
    }

    cv::projectPoints(cvPoints, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), cameraMatrix, distCoeffs, imagePoints);
}

cv::Scalar intensityToColor(float intensity) {
    int blue = std::max(0.0f, 255.0f - intensity*10);
    int red = std::min(255.0f, intensity*10);
    return cv::Scalar(blue, 0, red); // BGR格式
}

cv::Scalar zToColor(float z, float z_min, float z_max) {
    // 将z值归一化到[0, 1]区间
    float normalized = (z - z_min) / (z_max - z_min);

    // 将归一化的值映射到Hue值（色调），这里使用240到0的范围，对应从蓝色到红色
    // HSV中H的范围通常是0-360，但在OpenCV中Hue的范围是0-180
    float hue = 240 - normalized * 240;

    // 创建HSV颜色
    cv::Mat hsvColor(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));

    // 将HSV转换为RGB
    cv::Mat rgbColor;
    cvtColor(hsvColor, rgbColor, cv::COLOR_HSV2BGR);

    // 返回RGB颜色
    return cv::Scalar(rgbColor.at<cv::Vec3b>(0,0)[0], rgbColor.at<cv::Vec3b>(0,0)[1], rgbColor.at<cv::Vec3b>(0,0)[2]);
}

void drawPointsOnImage(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                       const std::vector<cv::Point2f>& points,
                       cv::Mat& image) {
    for (size_t i = 0; i < points.size(); i++) {
        // auto color = intensityToColor(cloud.points[i].intensity);
        auto color = zToColor(cloud.points[i].z, 0, 2500);
        cv::circle(image, points[i], 3, color, -1); // 根据强度填充颜色
    }
}


Eigen::Matrix4f createTransformMatrix() {
    
    float scale = 1000.0f;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = 0;
    transform(0, 1) = -1 * scale;
    transform(0, 2) = 0;
    transform(1, 0) = 0;
    transform(1, 1) = 0;
    transform(1, 2) = -1 * scale;
    transform(2, 0) = 1 * scale;
    transform(2, 1) = 0;
    transform(2, 2) = 0;
    return transform;
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
    rosHandle.param("frame_id", frame_id, std::string("livox_frame"));
	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/livox/lidar"));
    rosHandle.param("calibration_pc_pub_topic", topic_pc_pub, std::string("/livox_hikcamera_cal/pointcloud"));
	rosHandle.param("image_process_img_sub_topic", topic_img_sub, std::string("/hikcamera/img_stream"));
    rosHandle.param("calibration_img_pub_topic", topic_img_pub, std::string("/livox_hikcamera_cal/image"));
    rosHandle.param("pointcloud_process_corners_pub_topic", topic_pc_corners_sub, std::string("/livox_hikcamera_cal/pointcloud_corners"));
    rosHandle.param("image_process_corners_pub_topic", topic_img_corners_sub, std::string("/livox_hikcamera_cal/image_corners"));
    rosHandle.param("calibration_corners_pub_topic", topic_corners_pub, std::string("/livox_hikcamera_cal/calibration_corners"));



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


    PointCloudSubscriberPublisher pc_SUB_PUB(rosHandle, topic_pc_sub, topic_pc_pub);

    ImageSubscriberPublisher img_SUB_PUB(rosHandle, topic_img_sub, topic_img_pub);

    CornersPublisherSubscriber pc_corners_SUB_PUB(rosHandle, frame_id, topic_pc_corners_sub, topic_corners_pub);

    CornersPublisherSubscriber img_corners_SUB_PUB(rosHandle, frame_id, topic_img_corners_sub, topic_corners_pub);


    PointCloud2Proc pc_process(true);

    Draw3D d3d(arucoRealLength[0], 1, 1, 1, cameraMatrix, disCoffes);
    
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
			ROS_INFO("Waiting For Point Cloud Subscribe\n");
			continue;
		}

        auto img = img_SUB_PUB.getImage();

        if(img.empty())
        {
            ROS_INFO("Waiting For Image Subscribe\n");
            continue;
        }


        


        Eigen::Matrix4f transform = createTransformMatrix();

        pcl::PointCloud<pcl::PointXYZI>::Ptr axis_transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::transformPointCloud(*pc_process.getProcessedPointcloud(), *axis_transformedCloud, transform);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
        transformPointCloud(*axis_transformedCloud, *transformedCloud, R, t);

        pc_process.setCloud(transformedCloud);
        pc_process.PassThroughFilter("z", 0, 4000);

        std::vector<cv::Point2f> imagePoints;
        projectToImage(*pc_process.getProcessedPointcloud(), cameraMatrix, disCoffes, imagePoints);

        drawPointsOnImage(*pc_process.getProcessedPointcloud(), imagePoints, img);

        // 显示图像
        cv::imshow("Projected Points", img);
        int key = cv::waitKey(1); // 毫秒级延时，非阻塞
        if (key == 27) break; // 按 'ESC' 键退出
        if (key == 13)
        {
            std::vector<geometry_msgs::Point32> pc_corners_raw = pc_corners_SUB_PUB.getCornersPoints32();
            std::vector<geometry_msgs::Point32> img_corners_raw = img_corners_SUB_PUB.getCornersPoints32();

            if(pc_corners_raw.empty() || img_corners_raw.empty())
            {
                ROS_INFO("No Corners Found\n");
                continue;
            }

            TransformCorners(pc_corners_raw);
            TransformCorners(img_corners_raw);

            Eigen::Vector3f center_pc = computeCentroid(pc_corners_raw);
            Eigen::Vector3f center_img = computeCentroid(img_corners_raw);

            Eigen::MatrixXf pc_center_refer;
            Eigen::MatrixXf img_center_refer;

            alignPointsToCentroid(pc_corners_raw, center_pc, pc_center_refer);
            alignPointsToCentroid(img_corners_raw, center_img, img_center_refer);

            R = findRotation(pc_center_refer, img_center_refer);
            t = findTranslation(center_pc, center_img, R);

            std::cout << R << std::endl << t << std::endl;
        }


        
        

        rate.sleep();

    }

    return 0;
}

