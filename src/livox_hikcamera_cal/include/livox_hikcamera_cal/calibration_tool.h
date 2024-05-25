#ifndef _CALIBRATION_SOLVER_H_
#define _CALIBRATION_SOLVER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

using namespace std;

namespace livox_hikcamera_cal
{
    class CalTool
    {
        public:

            CalTool();
            ~CalTool();


            static void sortPointByNormalWorldFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal,
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            static void sortPointByNormalWorldFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            static void sortPointByNormalImgFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal,
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            static void sortPointByNormalImgFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));




            static void transformPointsImgToWorld(std::vector<geometry_msgs::Point32>& points);
            static void transformPointsWorldToImg(std::vector<geometry_msgs::Point32>& points);
            static Eigen::Vector3f computeCentroid(const std::vector<geometry_msgs::Point32>& points);
            static void alignPointsToCentroid(const std::vector<geometry_msgs::Point32>& points, const Eigen::Vector3f& centroid, Eigen::MatrixXf& points_matrix);
            static Eigen::Matrix3f findRotationByICP(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y);
            static Eigen::Vector3f findTranslation(const Eigen::Vector3f& centroidX, const Eigen::Vector3f& centroidY, const Eigen::Matrix3f& R);



            static int SolveSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output);


            static Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions);

            
        private:
        
            Eigen::Matrix3d R_;
            Eigen::Vector3d t_;
    };
}



#endif