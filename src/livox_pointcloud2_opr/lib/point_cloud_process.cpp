#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
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
#include "livox_pc2_opr/point_cloud_process.h"
#include <pcl/visualization/cloud_viewer.h> 
namespace livox_pc2_opr
{
    PointCloud2Proc::PointCloud2Proc() :    raw_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                            processed_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                            tree(new pcl::search::KdTree<pcl::PointXYZ>),
                                            normals(new pcl::PointCloud<pcl::Normal>),
                                            plane_coefficients(new pcl::ModelCoefficients),
                                            rect_corners_3d(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // this->raw_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // this->processed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // this->tree = pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
        // this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        // this->rect_corners_3d = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    PointCloud2Proc::PointCloud2Proc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :   raw_cloud(cloud),
                                                                                    processed_cloud(cloud),
                                                                                    tree(new pcl::search::KdTree<pcl::PointXYZ>),
                                                                                    normals(new pcl::PointCloud<pcl::Normal>),
                                                                                    plane_coefficients(new pcl::ModelCoefficients),
                                                                                    rect_corners_3d(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // this->raw_cloud = pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(cloud);
        // this->processed_cloud = pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(cloud),
        // this->tree = pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
        // this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        // this->rect_corners_3d = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    PointCloud2Proc::~PointCloud2Proc() {}

    int PointCloud2Proc::loadPointCloudFile(std::string file_name)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
        this->raw_cloud->clear();
        if (pcl::io::loadPCDFile(file_name, *this->raw_cloud) < 0)
        {
            PCL_ERROR("Failed to load point cloud!\n");
            return -1;
        }
        this->processedCloudUpdate(this->raw_cloud);
        // this->processed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(this->raw_cloud);
        
        return 0;
    }

    void PointCloud2Proc::resetCloud()
    {
        if(this->raw_cloud){
            printf("Point Cloud Re-setting Failed! Not allow empty point cloud reset");
            return;
        }
        this->processedCloudUpdate(this->raw_cloud);
    }
    void PointCloud2Proc::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        this->rawCloudUpdate(cloud);
        this->processedCloudUpdate(cloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2Proc::getRawPointcloud()
    {
        return this->raw_cloud;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2Proc::getProcessedPointcloud()
    {
        return this->processed_cloud;
    }
    std::vector<pcl::PointIndices> PointCloud2Proc::getClastersIndices()
    {
        return this->clusters;
    }
    pcl::PointIndices PointCloud2Proc::getClusterIndices(int index_of_cluster_indices)
    {
        return this->clusters[index_of_cluster_indices];
    }
    Eigen::Matrix4f PointCloud2Proc::getPCATransformMatrix()
    {
        return this->pca_transform_matrix;
    }
    cv::Point2f* PointCloud2Proc::getPCAPlaneRectCorners()
    {
        return this->rect_corners_2d;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2Proc::get3DRectCorners()
    {
        return this->rect_corners_3d;
    }


    PointCloud2Proc& PointCloud2Proc::boxFilter(Eigen::Vector4f min_point, Eigen::Vector4f max_point)
    {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        boxFilter.setInputCloud(this->processed_cloud);
        boxFilter.setMin(min_point);
        boxFilter.setMax(max_point);

        boxFilter.filter(*tempCloud);
        this->processedCloudUpdate(tempCloud);
        
        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::normalClusterExtraction()
    {
        int k_search = 50;
        float smoothness = 3.0 / 180.0 * M_PI;
        float curvature = 0.01;
        int number_of_neighbours = 30;
        int min_cluster_size = 100;
        int max_cluster_size = 25000;

        this->computeNormals(k_search);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);  
        reg.setMaxClusterSize(max_cluster_size);  
        reg.setSearchMethod(this->tree);    
        reg.setNumberOfNeighbours(number_of_neighbours);    
        reg.setInputCloud(this->processed_cloud);         
        reg.setInputNormals(this->normals);     
        reg.setSmoothnessThreshold(smoothness);  //smoothness
        reg.setCurvatureThreshold(curvature);     //curvature

        reg.extract(this->clusters);

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
        cluster_viewer.showCloud(colored_cloud);
        sleep(5);

        this->processedCloudUpdate(this->processed_cloud, this->clusters);
        
        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::normalClusterExtraction(int k_search, float smoothness = 3.0 / 180.0 * M_PI, float curvature = 0.1, int number_of_neighbours = 30,
                                                            int min_cluster_size = 100, int max_cluster_size = 25000)
    {
        this->computeNormals(k_search);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);  
        reg.setMaxClusterSize(max_cluster_size);  
        reg.setSearchMethod(this->tree);    
        reg.setNumberOfNeighbours(number_of_neighbours);    
        reg.setInputCloud(this->processed_cloud);         
        reg.setInputNormals(this->normals);     
        reg.setSmoothnessThreshold(smoothness);  //smoothness
        reg.setCurvatureThreshold(curvature);     //curvature

        reg.extract(this->clusters);

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
        cluster_viewer.showCloud(colored_cloud);
        sleep(5);

        this->processedCloudUpdate(this->processed_cloud, this->clusters);
        
        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::normalClusterExtraction(int (*ClustersIndexSelectorFunction)(std::vector<pcl::PointIndices>), int k_search = 50, float smoothness = 3.0 / 180.0 * M_PI, float curvature = 0.1,
                                                            int number_of_neighbours = 30, int min_cluster_size = 100, int max_cluster_size = 100000)
    {
        this->computeNormals(k_search);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);  
        reg.setMaxClusterSize(max_cluster_size);  
        reg.setSearchMethod(this->tree);    
        reg.setNumberOfNeighbours(number_of_neighbours);    
        reg.setInputCloud(this->processed_cloud);         
        reg.setInputNormals(normals);     
        reg.setSmoothnessThreshold(smoothness);  //smoothness
        reg.setCurvatureThreshold(curvature);     //curvature

        reg.extract(this->clusters);

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
        cluster_viewer.showCloud(colored_cloud);
        sleep(5);

        this->processedCloudUpdate(this->processed_cloud, this->clusters[ClustersIndexSelectorFunction(this->clusters)]);
    
        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::extractNearestCluster()
    {
        Eigen::Vector4f referencePoint = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0);
        pcl::PointIndices cluster = this->getNearestClusterIndices(this->clusters, referencePoint);    

        this->processedCloudUpdate(this->processed_cloud, cluster);

        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::extractNearestCluster(Eigen::Vector4f referencePoint)
    {
        pcl::PointIndices cluster = this->getNearestClusterIndices(this->clusters, referencePoint);   

        this->processedCloudUpdate(this->processed_cloud, cluster);

        return *this;
    }

// std::cout << this->clusters.size() << std::endl;
//         pcl::visualization::CloudViewer viewer("viewer");
// 		viewer.showCloud(this->processed_cloud);
// 		sleep(10);

    PointCloud2Proc& PointCloud2Proc::planeSegmentation()
    {
        int distance_threshold = 0.01;
        int max_iterations = 1000;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true); 
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);        
        seg.setDistanceThreshold(distance_threshold);    

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setInputCloud(this->processed_cloud);
        seg.segment(*inliers, *coefficients);

        // 提取内点
        extract.setInputCloud(this->processed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        this->processedCloudUpdate(this->processed_cloud);
            
        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::planeSegmentation(int distance_threshold, int max_iterations = 1000)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true); 
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);        
        seg.setDistanceThreshold(distance_threshold);    

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setInputCloud(this->processed_cloud);
        seg.segment(*inliers, *coefficients);

        // 提取内点
        extract.setInputCloud(this->processed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        this->plane_coefficients = pcl::ModelCoefficients::Ptr(coefficients);
        this->processedCloudUpdate(cloud_plane);
            
        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::planeProjection()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(this->plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::planeProjection(pcl::ModelCoefficients::Ptr plane_coefficients)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::planeProjection(Eigen::Vector4f coefficient)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        coefficients->values.resize(4);
        coefficients->values[0] = coefficient[0];
        coefficients->values[1] = coefficient[1];
        coefficients->values[2] = coefficient[2];
        coefficients->values[3] = coefficient[3];

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::pcaTransform()
    {
        Eigen::Matrix3f eigen_vector;
        Eigen::Vector4f mean_vector;
        this->computePCAMatrix(eigen_vector, mean_vector);
        this->pca_transform_matrix = this->computeTransformMatrix(eigen_vector, mean_vector);

        this->processedCloudTransform(this->processed_cloud, this->pca_transform_matrix);

        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::transform(Eigen::Matrix4f transform_matrix)
    {
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix)
    {
        Eigen::Matrix4f transform_matrix = this->computeTransformMatrix(rotation_matrix, translation_matrix);
        
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::findRectangleCornersInPCAPlane()
    {
        std::vector<cv::Point2f> pca_points;
        for (const auto& point : *this->processed_cloud) {
            pca_points.emplace_back(point.x, point.y);
        }

        this->pointcloud_rect_box = cv::minAreaRect(cv::Mat(pca_points));
        pointcloud_rect_box.points(this->rect_corners_2d);

        return *this;
    }

    PointCloud2Proc& PointCloud2Proc::transformCornersTo3D()
    {
        for (int i = 0; i < 4; i++) {
            Eigen::Vector4f pt_2d(this->rect_corners_2d[i].x, this->rect_corners_2d[i].y, 0, 1);
            Eigen::Vector4f pt_3d = this->pca_transform_matrix.inverse() * pt_2d;
            this->rect_corners_3d->push_back(pcl::PointXYZ(pt_3d[0], pt_3d[1], pt_3d[2]));
        }

        return *this;
    }
    PointCloud2Proc& PointCloud2Proc::transformCornersTo3D(Eigen::Matrix4f transform_matrix)
    {
        for (int i = 0; i < 4; i++) {
            Eigen::Vector4f pt_2d(this->rect_corners_2d[i].x, this->rect_corners_2d[i].y, 0, 1);
            Eigen::Vector4f pt_3d = transform_matrix * pt_2d;
            this->rect_corners_3d->push_back(pcl::PointXYZ(pt_3d[0], pt_3d[1], pt_3d[2]));
        }

        return *this;
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2Proc::extractNearestRectangleCorners()
    {
        pcl::PointCloud<pcl::PointXYZ> rect_corners;
        this->normalClusterExtraction();
        this->extractNearestCluster();
        this->planeSegmentation();
        this->planeProjection();
        this->pcaTransform();
        this->findRectangleCornersInPCAPlane();
        this->transformCornersTo3D();
        this->transform(this->pca_transform_matrix.inverse());

        return this->rect_corners_3d;
    }




    pcl::PointCloud<pcl::Normal>::Ptr PointCloud2Proc::computeNormals(int k_search = 50)
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(this->tree);
        normal_estimator.setInputCloud(this->processed_cloud);
        normal_estimator.setKSearch(k_search);
        normal_estimator.compute(*this->normals);

        return normals;
    }

    pcl::PointIndices PointCloud2Proc::getNearestClusterIndices(std::vector<pcl::PointIndices> clusters, 
                                                        Eigen::Vector4f referencePoint = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0))
    {
        float minDistance = std::numeric_limits<float>::max();
        int nearestPlaneIndex = -1;

        for (size_t i = 0; i < clusters.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*this->processed_cloud, clusters[i], *cluster);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            float distance = (centroid.head<3>() - referencePoint.head<3>()).norm(); // 计算距离
            if (distance < minDistance) {
                minDistance = distance;
                nearestPlaneIndex = i;
            }
        }
        return clusters[nearestPlaneIndex];
    }

    void PointCloud2Proc::computePCAMatrix(Eigen::Matrix3f& eigen_vector, Eigen::Vector4f& mean_vector)
    {
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(this->processed_cloud);
        eigen_vector = pca.getEigenVectors();
        mean_vector = pca.getMean();
    }

    Eigen::Matrix4f PointCloud2Proc::computeTransformMatrix(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0, 0) = rotation_matrix.transpose();  // 旋转
        transform.block<3,1>(0, 3) = -1.0 * (rotation_matrix.transpose() * translation_matrix.head<3>()); 
        
        return transform;
    }

    void PointCloud2Proc::rawCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        this->raw_cloud->clear();
        pcl::copyPointCloud(*cloud, *this->raw_cloud);
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr write_in_cloud(cloud);
        this->processed_cloud->clear();
        pcl::copyPointCloud(*write_in_cloud, *this->processed_cloud);
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr write_in_cloud(cloud);
        this->processed_cloud->clear();
        pcl::copyPointCloud(*write_in_cloud, cluster_indices, *this->processed_cloud);
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices> clusters_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr write_in_cloud(cloud);
        this->processed_cloud->clear();
        pcl::copyPointCloud(*write_in_cloud, clusters_indices, *this->processed_cloud);
    }
    void PointCloud2Proc::processedCloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr write_in_cloud(cloud);
        pcl::transformPointCloud(*write_in_cloud, *this->processed_cloud, transform_matrix);
    }

}