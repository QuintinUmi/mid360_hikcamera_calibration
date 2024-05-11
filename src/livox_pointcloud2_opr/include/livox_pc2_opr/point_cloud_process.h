#ifndef _POINT_CLOUD_PROCESS_H_
#define _POINT_CLOUD_PROCESS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/feature.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include "livox_pc2_opr/point_cloud_subscriber_publisher.h"
#include <pcl/visualization/cloud_viewer.h> 
namespace livox_pc2_opr
{

    class PointCloud2Proc
    {
        public:
            PointCloud2Proc();
            PointCloud2Proc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
            ~PointCloud2Proc();

            int loadPointCloudFile(std::string file_name);

            void resetCloud();
            void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr getRawPointcloud();
            pcl::PointCloud<pcl::PointXYZ>::Ptr getProcessedPointcloud();
            std::vector<pcl::PointIndices> getClastersIndices();
            pcl::PointIndices getClusterIndices(int index_of_cluster_indices);
            Eigen::Matrix4f getPCATransformMatrix();
            cv::Point2f* getPCAPlaneRectCorners();
            pcl::PointCloud<pcl::PointXYZ>::Ptr get3DRectCorners();


            PointCloud2Proc& boxFilter(Eigen::Vector4f min_point, Eigen::Vector4f max_point);
            PointCloud2Proc& normalClusterExtraction();
            PointCloud2Proc& normalClusterExtraction(int k_search, float smoothness, float curvature, int number_of_neighbours,
                                                    int min_cluster_size, int max_cluster_size);
            PointCloud2Proc& normalClusterExtraction(int (*ClustersIndexSelectorFunction)(std::vector<pcl::PointIndices>), int k_search, float smoothness, float curvature, 
                                                    int number_of_neighbours, int min_cluster_size, int max_cluster_size);
            PointCloud2Proc& extractNearestCluster();  
            PointCloud2Proc& extractNearestCluster(Eigen::Vector4f referencePoint);    
            PointCloud2Proc& planeSegmentation();   
            PointCloud2Proc& planeSegmentation(int max_iterations, int distance_threshold);
            PointCloud2Proc& planeProjection();
            PointCloud2Proc& planeProjection(pcl::ModelCoefficients::Ptr plane_coefficients);
            PointCloud2Proc& planeProjection(Eigen::Vector4f coefficient);
            PointCloud2Proc& pcaTransform();
            PointCloud2Proc& transform(Eigen::Matrix4f transform_matrix);
            PointCloud2Proc& transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix);
            PointCloud2Proc& findRectangleCornersInPCAPlane();
            PointCloud2Proc& transformCornersTo3D();
            PointCloud2Proc& transformCornersTo3D(Eigen::Matrix4f transform_matrix);

            pcl::PointCloud<pcl::PointXYZ>::Ptr extractNearestRectangleCorners();



            pcl::PointCloud<pcl::Normal>::Ptr computeNormals(int k_search);
            pcl::PointIndices getNearestClusterIndices(std::vector<pcl::PointIndices> clusters, Eigen::Vector4f referencePoint);
            void computePCAMatrix(Eigen::Matrix3f& eigen_vector, Eigen::Vector4f& mean_vector);
            Eigen::Matrix4f computeTransformMatrix(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix);
        
            void rawCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
            void processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
            void processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices cluster_indices);
            void processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices> clusters_indices);
            void processedCloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix);

        private:
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud;

            pcl::PointCloud<pcl::Normal>::Ptr normals;
            std::vector<pcl::PointIndices> clusters;

            pcl::ModelCoefficients::Ptr plane_coefficients;
            Eigen::Matrix4f pca_transform_matrix;
            cv::RotatedRect pointcloud_rect_box;
            cv::Point2f rect_corners_2d[4];
            pcl::PointCloud<pcl::PointXYZ>::Ptr rect_corners_3d;


            pcl::search::Search<pcl::PointXYZ>::Ptr tree;

        private:
            int k_search_;
            pcl::search::Search<pcl::PointXYZ>::Ptr search_method_;

    };



}


#endif