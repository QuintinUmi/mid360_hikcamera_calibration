#include <boost/filesystem.hpp>
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

// pcl::visualization::CloudViewer viewer("viewer");

namespace livox_pc2_opr
{
    PointCloud2Proc::PointCloud2Proc() :    raw_cloud(new pcl::PointCloud<pcl::PointXYZI>),
                                            processed_cloud(new pcl::PointCloud<pcl::PointXYZI>),
                                            tree(boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>)),
                                            normals(boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>)),
                                            pca_transform_matrix(Eigen::Matrix4f::Identity()),
                                            plane_coefficients(new pcl::ModelCoefficients),
                                            rect_corners_3d(new pcl::PointCloud<pcl::PointXYZI>),
                                            no_cloud(new pcl::PointCloud<pcl::PointXYZI>)
    {
        // this->raw_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // this->processed_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // this->tree = pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
        // this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        // this->rect_corners_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }
    
    PointCloud2Proc::PointCloud2Proc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) :  raw_cloud(cloud),
                                                                                    processed_cloud(cloud),
                                                                                    tree(boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>)),
                                                                                    normals(boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>)),
                                                                                    pca_transform_matrix(Eigen::Matrix4f::Identity()),
                                                                                    plane_coefficients(new pcl::ModelCoefficients),
                                                                                    rect_corners_3d(new pcl::PointCloud<pcl::PointXYZI>),
                                                                                    no_cloud(new pcl::PointCloud<pcl::PointXYZI>)
    {
        // this->raw_cloud = pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(cloud);
        // this->processed_cloud = pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(cloud),
        // this->tree = pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
        // this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        // this->rect_corners_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    PointCloud2Proc::~PointCloud2Proc() {}

    int PointCloud2Proc::loadPointCloudFile(std::string file_name)
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZI>);
        this->raw_cloud->clear();
        if (pcl::io::loadPCDFile(file_name, *this->raw_cloud) < 0)
        {
            PCL_ERROR("Failed to load point cloud!\n");
            return -1;
        }
        this->resetCloud();
        
        return 0;
    }

    void PointCloud2Proc::resetCloud()
    {
        if(!this->raw_cloud){
            printf("Point Cloud Re-setting Failed! Not allow empty point cloud reset\n");
            return;
        }
        this->processedCloudUpdate(this->raw_cloud);

        this->tree = pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
        this->normals = boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>),
        this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        this->rect_corners_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        this->pca_transform_matrix = Eigen::Matrix4f::Identity();
        this->clusters.clear();
        this->rect_corners_3d->clear();
    }
    void PointCloud2Proc::setCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        this->rawCloudUpdate(cloud);
        this->resetCloud();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::getRawPointcloud()
    {
        return this->raw_cloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::getProcessedPointcloud()
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::get3DRectCorners()
    {
        return this->rect_corners_3d;
    }


    void PointCloud2Proc::boxFilter(Eigen::Vector4f min_point, Eigen::Vector4f max_point)
    {
        pcl::CropBox<pcl::PointXYZI> boxFilter;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
        boxFilter.setInputCloud(this->processed_cloud);
        boxFilter.setMin(min_point);
        boxFilter.setMax(max_point);

        boxFilter.filter(*tempCloud);
        this->processedCloudUpdate(tempCloud);
        
        return;
    }

    std::vector<pcl::PointIndices> PointCloud2Proc::normalClusterExtraction(float smoothness, float curvature, int number_of_neighbours, int k_search, 
                                                            int min_cluster_size, int max_cluster_size)
    {
        this->computeNormals(k_search);

        pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);  
        reg.setMaxClusterSize(max_cluster_size);  
        reg.setSearchMethod(this->tree);    
        reg.setNumberOfNeighbours(number_of_neighbours);    
        reg.setInputCloud(this->processed_cloud);         
        reg.setInputNormals(this->normals);     
        reg.setSmoothnessThreshold(smoothness);  //smoothness
        reg.setCurvatureThreshold(curvature);     //curvature

        this->clusters.clear();
        reg.extract(this->clusters);

        return this->clusters;
    }
    std::vector<pcl::PointIndices> PointCloud2Proc::normalClusterExtraction(int (*ClustersIndexSelectorFunction)(std::vector<pcl::PointIndices>), float smoothness, float curvature, int k_search, 
                                                            int number_of_neighbours, int min_cluster_size, int max_cluster_size)
    {
        this->computeNormals(k_search);

        pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);  
        reg.setMaxClusterSize(max_cluster_size);  
        reg.setSearchMethod(this->tree);    
        reg.setNumberOfNeighbours(number_of_neighbours);    
        reg.setInputCloud(this->processed_cloud);         
        reg.setInputNormals(normals);     
        reg.setSmoothnessThreshold(smoothness);  //smoothness
        reg.setCurvatureThreshold(curvature);     //curvature

        this->clusters.clear();
        reg.extract(this->clusters);
        
        return this->clusters;
    }

    pcl::PointIndices PointCloud2Proc::extractNearestClusterCloud(Eigen::Vector4f referencePoint)
    {
        pcl::PointIndices cluster = this->computeNearestClusterIndices(this->processed_cloud, this->clusters, referencePoint);   

        this->processedCloudUpdate(this->processed_cloud, cluster);

        return cluster;
    }

    pcl::PointIndices PointCloud2Proc::planeSegmentation(float distance_threshold, int max_iterations)
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true); 
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);        
        seg.setDistanceThreshold(distance_threshold);    

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointIndices inliers_;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setInputCloud(this->processed_cloud);
        seg.segment(*inliers, *coefficients);

        inliers_.indices = inliers->indices;
        if(inliers_.indices.size() == 0)
        {
            // ROS_INFO("No Plane Can Find!\n");
            return inliers_;
        }

        extract.setInputCloud(this->processed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // ROS_INFO("Find Plane!\n");

        this->plane_coefficients->header = coefficients->header;
        this->plane_coefficients->values = coefficients->values;
        this->processedCloudUpdate(cloud_plane);
            
        return inliers_;
    }

    void PointCloud2Proc::planeProjection()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(this->plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }
    void PointCloud2Proc::planeProjection(pcl::ModelCoefficients::Ptr plane_coefficients)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }
    void PointCloud2Proc::planeProjection(Eigen::Vector4f coefficient)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        coefficients->values.resize(4);
        coefficients->values[0] = coefficient[0];
        coefficients->values[1] = coefficient[1];
        coefficients->values[2] = coefficient[2];
        coefficients->values[3] = coefficient[3];

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }

    void PointCloud2Proc::pcaTransform()
    {
        Eigen::Matrix3f eigen_vector;
        Eigen::Vector4f mean_vector;
        this->computePCAMatrix(eigen_vector, mean_vector);
        this->pca_transform_matrix = this->computeTransformMatrix(eigen_vector, mean_vector);

        this->processedCloudTransform(this->processed_cloud, this->pca_transform_matrix);

        return;
    }

    void PointCloud2Proc::transform(Eigen::Matrix4f transform_matrix)
    {
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return;
    }
    void PointCloud2Proc::transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix)
    {
        Eigen::Matrix4f transform_matrix = this->computeTransformMatrix(rotation_matrix, translation_matrix);
        
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return;
    }

    void PointCloud2Proc::findRectangleCornersInPCAPlane()
    {
        std::vector<cv::Point2f> pca_points;
        for (const auto& point : *this->processed_cloud) {
            pca_points.emplace_back(point.x, point.y);
        }

        this->pointcloud_rect_box = cv::minAreaRect(cv::Mat(pca_points));
        pointcloud_rect_box.points(this->rect_corners_2d);

        return;
    }

    void PointCloud2Proc::transformCornersTo3D()
    {
        for (int i = 0; i < 4; i++) {
            Eigen::Vector4f pt_2d(this->rect_corners_2d[i].x, this->rect_corners_2d[i].y, 0, 1);
            Eigen::Vector4f pt_3d = this->pca_transform_matrix.inverse() * pt_2d;
            pcl::PointXYZI point_3d;
            point_3d.x = pt_3d[0];
            point_3d.y = pt_3d[1];
            point_3d.z = pt_3d[2];
            point_3d.intensity = 0;
            this->rect_corners_3d->push_back(point_3d);
        }

        return;
    }
    void PointCloud2Proc::transformCornersTo3D(Eigen::Matrix4f transform_matrix)
    {
        for (int i = 0; i < 4; i++) {
            Eigen::Vector4f pt_2d(this->rect_corners_2d[i].x, this->rect_corners_2d[i].y, 0, 1);
            Eigen::Vector4f pt_3d = transform_matrix * pt_2d;
            pcl::PointXYZI point_3d;
            point_3d.x = pt_3d[0];
            point_3d.y = pt_3d[1];
            point_3d.z = pt_3d[2];
            point_3d.intensity = 0;
            this->rect_corners_3d->push_back(point_3d);
        }

        return;
    }



    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::extractNearestRectangleCorners()
    {
        
        if(this->normalClusterExtraction().size() == 0)
        {
            return this->NOCLOUD();
        }
        if(this->extractNearestClusterCloud().indices.size() == 0)
        {
            return this->NOCLOUD();
        }
        if(this->planeSegmentation().indices.size() == 0)
        {
            return this->NOCLOUD();
        }
        // this->planeProjection();
        this->pcaTransform();
        this->findRectangleCornersInPCAPlane();
        this->transformCornersTo3D();
        this->processedCloudUpdate(this->raw_cloud, this->computeNearestClusterIndices(this->raw_cloud, this->clusters));

        return this->rect_corners_3d;
        
        
        
    }




    pcl::PointCloud<pcl::Normal>::Ptr PointCloud2Proc::computeNormals(int k_search = 50)
    {
        // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(this->tree);
        normal_estimator.setInputCloud(this->processed_cloud);
        normal_estimator.setKSearch(k_search);
        normal_estimator.compute(*this->normals);

        return this->normals;
    }

    pcl::PointIndices PointCloud2Proc::computeNearestClusterIndices(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, std::vector<pcl::PointIndices> input_clusters, 
                                                        Eigen::Vector4f referencePoint)
    {
        float minDistance = std::numeric_limits<float>::max();
        int nearestPlaneIndex = -1; 

        if(input_clusters.size() <= 1)
        {
            nearestPlaneIndex = 0;
            return input_clusters[nearestPlaneIndex];
        }

        for (size_t i = 0; i < input_clusters.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*input_cloud, input_clusters[i], *cluster);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            float distance = (centroid.head<3>() - referencePoint.head<3>()).norm(); // 计算距离
            if (distance < minDistance) {
                minDistance = distance;
                nearestPlaneIndex = i;
            }
        }
        return input_clusters[nearestPlaneIndex];
    }

    void PointCloud2Proc::computePCAMatrix(Eigen::Matrix3f& eigen_vector, Eigen::Vector4f& mean_vector)
    {
        pcl::PCA<pcl::PointXYZI> pca;
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

    void PointCloud2Proc::rawCloudUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        this->raw_cloud->clear();
        pcl::copyPointCloud(*cloud, *this->raw_cloud);
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        if(cloud == this->processed_cloud)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            this->processed_cloud->clear();
            pcl::copyPointCloud(*write_in_cloud, *this->processed_cloud);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::copyPointCloud(*cloud, *this->processed_cloud);
        }
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndices cluster_indices)
    {
        if(cloud == this->processed_cloud)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            this->processed_cloud->clear();
            pcl::copyPointCloud(*write_in_cloud, cluster_indices, *this->processed_cloud);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::copyPointCloud(*cloud, cluster_indices, *this->processed_cloud);
        }
    }
    void PointCloud2Proc::processedCloudUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointIndices> clusters_indices)
    {
        if(cloud == this->processed_cloud)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            this->processed_cloud->clear();
            pcl::copyPointCloud(*write_in_cloud, clusters_indices, *this->processed_cloud);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::copyPointCloud(*cloud, clusters_indices, *this->processed_cloud);
        }
    }
    void PointCloud2Proc::processedCloudTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Matrix4f transform_matrix)
    {
        if(cloud == this->processed_cloud)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr write_in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            pcl::transformPointCloud(*write_in_cloud, *this->processed_cloud, transform_matrix);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::transformPointCloud(*cloud, *this->processed_cloud, transform_matrix);
        }       
    }

    std::vector<std::string> PointCloud2Proc::iterateFilesFromPath(std::string folderPath)
    { 

        std::vector<std::string> file_names;

        boost::filesystem::path dir(folderPath);
        if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
            for (const auto& entry : boost::filesystem::directory_iterator(dir)) {
                std::cout << entry.path().string() << '\n';
                file_names.emplace_back(entry.path().string());
            }
        } else {
            std::cerr << "Directory does not exist\n";
        }

        return file_names;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2Proc::NOCLOUD()
    {
        return no_cloud;
    }

}