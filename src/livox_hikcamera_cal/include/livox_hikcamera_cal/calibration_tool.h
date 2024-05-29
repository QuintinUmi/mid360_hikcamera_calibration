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

            template<typename PointT>
            static void sortPointByNormalWorldFrame(typename pcl::PointCloud<PointT>::Ptr points, const Eigen::Vector3f& normal,
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            static void sortPointByNormalWorldFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            template<typename PointT>
            static void sortPointByNormalImgFrame(typename pcl::PointCloud<PointT>::Ptr points, const Eigen::Vector3f& normal,
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));
            static void sortPointByNormalImgFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                            bool negetive=false, const Eigen::Vector3f& ref_point=Eigen::Vector3f(0.0, 0.0, 0.0));




            static void transformPointsImgToWorld(std::vector<geometry_msgs::Point32>& points);
            static void transformPointsWorldToImg(std::vector<geometry_msgs::Point32>& points);
            static Eigen::Vector3f computeCentroid(const std::vector<geometry_msgs::Point32>& points);
            static void alignPointsToCentroid(const std::vector<geometry_msgs::Point32>& points, const Eigen::Vector3f& centroid, Eigen::MatrixXf& points_matrix);
            static Eigen::Matrix3f findRotationByICP(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y);
            static Eigen::Vector3f findTranslation(const Eigen::Vector3f& centroidX, const Eigen::Vector3f& centroidY, const Eigen::Matrix3f& R);


            template<typename PointT>
            static int SolveSVD(typename pcl::PointCloud<PointT>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output);


            static Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions);

            
            template<typename PointT>
            static float pointToLineDistance(const PointT& point, const geometry_msgs::Point32& a, const geometry_msgs::Point32& b); 
            static float triangleArea(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3);
            static float quadrilateralArea(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3, const geometry_msgs::Point32& p4);
            template<typename PointT>
            static void removeBoundingBoxOutliers(const typename pcl::PointCloud<PointT>::Ptr cloud, const std::vector<geometry_msgs::Point32>& corners);




            struct Line {
                cv::Point2f start, end;

                double distanceToPoint(cv::Point2f point) const{
                    cv::Point2f lineVec = end - start;
                    cv::Point2f pointVec = point - start;

                    float lineLen = cv::norm(lineVec);
                    cv::Point2f lineUnitVec = lineVec / lineLen;

                    float projectedLength = pointVec.dot(lineUnitVec);
                    projectedLength = std::max(0.0f, std::min(lineLen, projectedLength));
                    cv::Point2f closestPoint = start + lineUnitVec * projectedLength;
                    return cv::norm(point - closestPoint);
                }
            };

            template<typename PointT>
            static void computeReprojectionErrorsInPixels(
                                            const typename pcl::PointCloud<PointT>::Ptr objectPoints,
                                            const std::vector<geometry_msgs::Point32>& imageCorners,
                                            const Eigen::Matrix3f& R,
                                            const Eigen::Vector3f& t,
                                            const cv::Mat& cameraMatrix,
                                            const cv::Mat& distCoeffs,
                                            double& meanError,  
                                            double& stdDev);
            template<typename PointT>                             
            static void computeReprojectionErrorsInPixels(
                                            const typename pcl::PointCloud<PointT>::Ptr objectPoints,
                                            const std::vector<std::vector<geometry_msgs::Point32>>& imageCorners,
                                            const Eigen::Matrix3f& R,
                                            const Eigen::Vector3f& t,
                                            const cv::Mat& cameraMatrix,
                                            const cv::Mat& distCoeffs,
                                            double& meanError,  
                                            double& stdDev);
            
        private:
        
            Eigen::Matrix3d R_;
            Eigen::Vector3d t_;
    };
}



#include "../lib/calibration_tool.tpp"

#endif