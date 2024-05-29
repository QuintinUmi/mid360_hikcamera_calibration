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
#include <pcl/surface/concave_hull.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include <pcl/visualization/cloud_viewer.h> 


namespace livox_hikcamera_cal
{

    namespace pointcloud2_opr
    {

        template<typename PointT>
        class PointCloud2Proc
        {
        public:
            enum class OptimizationMethod
            {
                None,
                AdjustCentroid,
                AngleAtCentroid
            };

        public:
            PointCloud2Proc(bool remove_origin_point=false);
            PointCloud2Proc(typename pcl::PointCloud<PointT>::Ptr cloud, bool remove_origin_point=false);
            ~PointCloud2Proc();

            int loadPointCloudFile(std::string file_name);

            void resetCloud();
            void setCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
            void setRemoveOriginPoint(bool remove_origin_point=true);

            typename pcl::PointCloud<PointT>::Ptr getRawPointcloud();
            typename pcl::PointCloud<PointT>::Ptr getProcessedPointcloud();
            std::vector<pcl::PointIndices> getClastersIndices();
            pcl::PointIndices getClusterIndices(int index_of_cluster_indices);
            pcl::ModelCoefficients getPlaneCoefficients();
            Eigen::Vector3f getPlaneNormals();
            Eigen::Matrix4f getPCATransformMatrix();
            cv::Point2f* getPCAPlaneRectCorners();
            typename pcl::PointCloud<PointT>::Ptr get3DRectCorners();

            typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColorPointCloudIntensity();

            void PassThroughFilter(std::string axis, float min, float max);
            void boxFilter(Eigen::Vector4f min_point, Eigen::Vector4f max_point, bool negative=false);
            void boxFilter(Eigen::Vector3f box_center, float length_x, float length_y, float length_z,
                            float angle_x=0.0, float angle_y=0.0, float angle_z=0.0, bool negative=false);
            std::vector<pcl::PointIndices> normalClusterExtraction(float smoothness = 3.0 / 180.0 * M_PI, float curvature = 0.05, int k_search = 90, 
                                                    int number_of_neighbours = 30, int min_cluster_size = 100, int max_cluster_size = 25000);
            std::vector<pcl::PointIndices> normalClusterExtraction(int (*ClustersIndexSelectorFunction)(std::vector<pcl::PointIndices>), 
                                                    float smoothness = 3.0 / 180.0 * M_PI, float curvature = 0.05, int k_search = 90, 
                                                    int number_of_neighbours = 30, int min_cluster_size = 100, int max_cluster_size = 25000);
            pcl::PointIndices extractNearestClusterCloud(Eigen::Vector4f referencePoint = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0));    
            int statisticalOutlierFilter(int mean_k=50, float stddev_mul=1.0);
            pcl::PointIndices planeSegmentation(float distance_threshold = 0.1, int max_iterations = 1000);
            void planeProjection();
            void planeProjection(pcl::ModelCoefficients::Ptr plane_coefficients);
            void planeProjection(Eigen::Vector4f coefficient);
            void pcaTransform();
            void scaleTo(float scale);
            void transform(Eigen::Matrix4f transform_matrix);
            void transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector3f translation_matrix);
            void transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix);
            void transformWorldToImg();
            void transformImgToWorld();
            void findRectangleCornersInPCAPlane();
            void optimizeRectangleAdjustCentroid(float constraint_width, float constraint_height, float optimize_offset_ratio = 0.05, float optimize_precision = 0.001);
            void optimizeRectangleAngleAtCentroid(float constraint_width, float constraint_height, float angle_range_ratio = 0.05, float angle_precision = 0.001);
            void transformCornersTo3D();
            void transformCornersTo3D(Eigen::Matrix4f transform_matrix);
            void extractConcaveHull(double alpha=0.1);
            void extractConvexHull();

            typename pcl::PointCloud<PointT>::Ptr extractNearestRectangleCorners(bool useStatisticalOutlierFilter=false, OptimizationMethod optimization_method=OptimizationMethod::AngleAtCentroid,
                                                                                float constraint_width = 0.0, float constraint_height = 0.0, 
                                                                                float optimize_range_ratio = (0.05F), float optimize_precision = (0.001F));

            typename pcl::PointCloud<PointT>::Ptr getFilterBoxCorners(Eigen::Vector4f min_point, Eigen::Vector4f max_point);
            typename pcl::PointCloud<PointT>::Ptr getFilterBoxCorners(Eigen::Vector3f box_center, float length_x, float length_y, float length_z,
                                                                        float angle_x=0.0, float angle_y=0.0, float angle_z=0.0);
            pcl::PointCloud<pcl::Normal>::Ptr computeNormals(int k_search=120);
            pcl::PointIndices computeNearestClusterIndices(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices> clusters, Eigen::Vector4f referencePoint = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0));
            void computePCAMatrix(Eigen::Matrix3f& eigen_vector, Eigen::Vector4f& mean_vector);
            Eigen::Matrix4f computeTransformMatrix(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix);
            Eigen::Matrix4f computeTransformMatrixWorldToImg();
            Eigen::Matrix4f computeTransformMatrixImgToWorld();
            void sortPointByNormal(typename pcl::PointCloud<PointT>::Ptr points, const Eigen::Vector3f& normal);

            typename pcl::PointCloud<PointT>::Ptr calculateConcaveHull(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, double alpha=0.1);
            typename pcl::PointCloud<PointT>::Ptr calculateConvexHull(const typename pcl::PointCloud<PointT>::Ptr &input_cloud);

        
            void rawCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud);
            void processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud);
            void processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices cluster_indices);
            void processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices> clusters_indices);
            void processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::Indices indices);

            void processedCloudTransform(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Matrix4f transform_matrix);

            std::vector<std::string> iterateFilesFromPath(std::string folderPath);
            void removeOriginPoint();

            typename pcl::PointCloud<PointT>::Ptr NOCLOUD();

        private:
            typename pcl::PointCloud<PointT>::Ptr raw_cloud;
            typename pcl::PointCloud<PointT>::Ptr processed_cloud;

            pcl::PointCloud<pcl::Normal>::Ptr normals;
            std::vector<pcl::PointIndices> clusters;

            pcl::ModelCoefficients::Ptr plane_coefficients;
            Eigen::Vector3f plane_normals;
            Eigen::Matrix4f pca_transform_matrix;
            std::vector<cv::Point2f> pca_points_;
            cv::RotatedRect pointcloud_rect_box;
            cv::Point2f rect_corners_2d[4];
            typename pcl::PointCloud<PointT>::Ptr rect_corners_3d;

            typename pcl::PointCloud<PointT>::Ptr no_cloud;


            pcl::search::Search<pcl::PointXYZI>::Ptr tree;

        private:
            bool remove_origin_point_;
            int k_search_;
            pcl::search::Search<pcl::PointXYZI>::Ptr search_method_;

            bool isPointInQuad(const cv::Point2f testCorners[4], const cv::Point2f& P) 
            {
                return isPointInTriangle(testCorners[0], testCorners[1], testCorners[2], P) || isPointInTriangle(testCorners[2], testCorners[3], testCorners[0], P);
            }

            bool isPointInTriangle(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C, const cv::Point2f& P) 
            {
                cv::Point2f AB = B - A;
                cv::Point2f BC = C - B;
                cv::Point2f CA = A - C;

                cv::Point2f AP = P - A;
                cv::Point2f BP = P - B;
                cv::Point2f CP = P - C;

                float cross1 = AB.x * AP.y - AB.y * AP.x;
                float cross2 = BC.x * BP.y - BC.y * BP.x;
                float cross3 = CA.x * CP.y - CA.y * CP.x;

                bool has_neg = (cross1 < 0) || (cross2 < 0) || (cross3 < 0);
                bool has_pos = (cross1 > 0) || (cross2 > 0) || (cross3 > 0);

                return !(has_neg && has_pos);
            }

        };



    }

}







#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/feature.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <pcl/surface/mls.h>



#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h"
#include <pcl/visualization/cloud_viewer.h> 

// pcl::visualization::CloudViewer viewer("viewer");

namespace livox_hikcamera_cal::pointcloud2_opr
{
    template<typename PointT>
    PointCloud2Proc<PointT>::PointCloud2Proc(bool remove_origin_point) :    
                                    raw_cloud(new pcl::PointCloud<PointT>),
                                    processed_cloud(new pcl::PointCloud<PointT>),
                                    tree(boost::shared_ptr<pcl::search::Search<PointT>>(new pcl::search::KdTree<PointT>)),
                                    normals(boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>)),
                                    pca_transform_matrix(Eigen::Matrix4f::Identity()),
                                    plane_coefficients(new pcl::ModelCoefficients),
                                    rect_corners_3d(new pcl::PointCloud<PointT>),
                                    no_cloud(new pcl::PointCloud<PointT>),
                                    remove_origin_point_(remove_origin_point) {}
    
    template<typename PointT>
    PointCloud2Proc<PointT>::PointCloud2Proc(typename pcl::PointCloud<PointT>::Ptr cloud, bool remove_origin_point) :  
                                    tree(boost::shared_ptr<pcl::search::Search<PointT>>(new pcl::search::KdTree<PointT>)),
                                    normals(boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>)),
                                    pca_transform_matrix(Eigen::Matrix4f::Identity()),
                                    plane_coefficients(new pcl::ModelCoefficients),
                                    rect_corners_3d(new pcl::PointCloud<PointT>),
                                    no_cloud(new pcl::PointCloud<PointT>),
                                    remove_origin_point_(remove_origin_point)
    {
        this->setCloud(cloud);
    }

    template<typename PointT>
    PointCloud2Proc<PointT>::~PointCloud2Proc() {}

    template<typename PointT>
    int PointCloud2Proc<PointT>::loadPointCloudFile(std::string file_name)
    {
        // typename pcl::PointCloud<PointT>::Ptr rawCloud(new typename pcl::PointCloud<PointT>);
        this->raw_cloud->clear();
        if (pcl::io::loadPCDFile(file_name, *this->raw_cloud) < 0)
        {
            PCL_ERROR("Failed to load point cloud!\n");
            return -1;
        }
        this->resetCloud();
        
        return 0;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::resetCloud()
    {
        if(!this->raw_cloud){
            printf("Point Cloud Re-setting Failed! Not allow empty point cloud reset\n");
            return;
        }
        this->processedCloudUpdate(this->raw_cloud);
        this->removeOriginPoint();

        this->tree = pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
        this->normals = boost::shared_ptr<pcl::PointCloud<pcl::Normal>>(new pcl::PointCloud<pcl::Normal>),
        this->plane_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        this->rect_corners_3d = typename pcl::PointCloud<PointT>::Ptr(new typename pcl::PointCloud<PointT>);
        this->pca_transform_matrix = Eigen::Matrix4f::Identity();
        this->clusters.clear();
        this->rect_corners_3d->clear();
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::setCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        this->rawCloudUpdate(cloud);
        this->resetCloud();
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::setRemoveOriginPoint(bool remove_origin_point)
    {
        this->remove_origin_point_ = remove_origin_point;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::getRawPointcloud()
    {
        return this->raw_cloud;
    }
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::getProcessedPointcloud()
    {
        return this->processed_cloud;
    }
    template<typename PointT>
    std::vector<pcl::PointIndices> PointCloud2Proc<PointT>::getClastersIndices()
    {
        return this->clusters;
    }
    template<typename PointT>
    pcl::PointIndices PointCloud2Proc<PointT>::getClusterIndices(int index_of_cluster_indices)
    {
        return this->clusters[index_of_cluster_indices];
    }
    template<typename PointT>
    pcl::ModelCoefficients PointCloud2Proc<PointT>::getPlaneCoefficients()
    {
        return *this->plane_coefficients;
    }
    template<typename PointT>
    Eigen::Vector3f PointCloud2Proc<PointT>::getPlaneNormals()
    {
        return this->plane_normals;
    }
    template<typename PointT>
    Eigen::Matrix4f PointCloud2Proc<PointT>::getPCATransformMatrix()
    {
        return this->pca_transform_matrix;
    }
    template<typename PointT>
    cv::Point2f* PointCloud2Proc<PointT>::getPCAPlaneRectCorners()
    {
        return this->rect_corners_2d;
    }
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::get3DRectCorners()
    {
        return this->rect_corners_3d;
    }

    template<typename PointT>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud2Proc<PointT>::getColorPointCloudIntensity() 
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (this->processed_cloud->empty())
            return colored_cloud;

        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = -std::numeric_limits<float>::max();

        colored_cloud->reserve(this->processed_cloud->size());
        for (const auto& point : this->processed_cloud->points) 
        {
            min_intensity = std::min(min_intensity, point.intensity);
            max_intensity = std::max(max_intensity, point.intensity);
        }

        float range = max_intensity - min_intensity;
        if (range == 0.0f) range = 1.0f;  

        for (const auto& point : this->processed_cloud->points) {
            uint8_t intensity_value = static_cast<uint8_t>((point.intensity - min_intensity) / range * 255.0);
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = colored_point.g = colored_point.b = intensity_value;  
            colored_cloud->push_back(colored_point);
        }

        return colored_cloud;
    }


    template<typename PointT>
    void PointCloud2Proc<PointT>::PassThroughFilter(std::string axis, float min, float max)
    {
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(this->processed_cloud);          
        pass.setFilterFieldName(axis);      
        pass.setFilterLimits(0.0, FLT_MAX); 

        typename pcl::PointCloud<PointT>::Ptr tempCloud(new typename pcl::PointCloud<PointT>);
        pass.filter(*tempCloud);
        this->processedCloudUpdate(tempCloud);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::boxFilter(Eigen::Vector4f min_point, Eigen::Vector4f max_point, bool negetive)
    {
        pcl::CropBox<pcl::PointXYZI> boxFilter;
        typename pcl::PointCloud<PointT>::Ptr tempCloud(new typename pcl::PointCloud<PointT>);
        boxFilter.setInputCloud(this->processed_cloud);
        boxFilter.setMin(min_point);
        boxFilter.setMax(max_point);
        boxFilter.setNegative(negetive);
    
        boxFilter.filter(*tempCloud);
        this->processedCloudUpdate(tempCloud);
        
        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::boxFilter(Eigen::Vector3f box_center, float length_x, float length_y, float length_z,
                                    float angle_x, float angle_y, float angle_z, bool negetive)
    {
        pcl::CropBox<pcl::PointXYZI> boxFilter;

        boxFilter.setMin(Eigen::Vector4f(-length_x/2, -length_y/2, -length_z/2, 1.0));
        boxFilter.setMax(Eigen::Vector4f(length_x/2, length_y/2, length_z/2, 1.0));

        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(angle_x / 180 * M_PI, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(angle_y / 180 * M_PI, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(angle_z / 180 * M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f rotate(q);
        
        Eigen::Translation3f translate_to_center(box_center);
        
        boxFilter.setTransform((translate_to_center * rotate).inverse());

        boxFilter.setNegative(negetive);

        typename pcl::PointCloud<PointT>::Ptr tempCloud(new typename pcl::PointCloud<PointT>);
        boxFilter.setInputCloud(this->processed_cloud);
        boxFilter.filter(*tempCloud);
        this->processedCloudUpdate(tempCloud);
        
        return;
    }

    template<typename PointT>
    std::vector<pcl::PointIndices> PointCloud2Proc<PointT>::normalClusterExtraction(float smoothness, float curvature, int number_of_neighbours, int k_search, 
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
    template<typename PointT>
    std::vector<pcl::PointIndices> PointCloud2Proc<PointT>::normalClusterExtraction(int (*ClustersIndexSelectorFunction)(std::vector<pcl::PointIndices>), float smoothness, float curvature, int k_search, 
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

    template<typename PointT>
    pcl::PointIndices PointCloud2Proc<PointT>::extractNearestClusterCloud(Eigen::Vector4f referencePoint)
    {
        pcl::PointIndices cluster = this->computeNearestClusterIndices(this->processed_cloud, this->clusters, referencePoint);   

        this->processedCloudUpdate(this->processed_cloud, cluster);

        return cluster;
    }

    template<typename PointT>
    int PointCloud2Proc<PointT>::statisticalOutlierFilter(int mean_k, float stddev_mul)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new typename pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        pcl::Indices indices;
        
        sor.setInputCloud(this->processed_cloud);
        sor.setMeanK(mean_k);  
        sor.setStddevMulThresh(stddev_mul); 
        sor.setNegative(false);
        sor.filter(*cloud_filtered);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        if(cloud_filtered->size() == 0)
        {
            return 0;
        }

        this->processedCloudUpdate(cloud_filtered);

        return cloud_filtered->size();
    }

    template<typename PointT>
    pcl::PointIndices PointCloud2Proc<PointT>::planeSegmentation(float distance_threshold, int max_iterations)
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true); 
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);        
        seg.setDistanceThreshold(distance_threshold);    

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        typename pcl::PointCloud<PointT>::Ptr cloud_plane(new typename pcl::PointCloud<PointT>);

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



        Eigen::Vector3f normal_(Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]).normalized());

        if (coefficients->values[3] < 0) 
        {
            normal_ = -normal_;
        }

        this->plane_normals = normal_;
            
        return inliers_;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::planeProjection()
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_projected(new typename pcl::PointCloud<PointT>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(this->plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::planeProjection(pcl::ModelCoefficients::Ptr plane_coefficients)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_projected(new typename pcl::PointCloud<PointT>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::planeProjection(Eigen::Vector4f coefficient)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        coefficients->values.resize(4);
        coefficients->values[0] = coefficient[0];
        coefficients->values[1] = coefficient[1];
        coefficients->values[2] = coefficient[2];
        coefficients->values[3] = coefficient[3];

        typename pcl::PointCloud<PointT>::Ptr cloud_projected(new typename pcl::PointCloud<PointT>);

        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->processed_cloud);
        proj.setModelCoefficients(plane_coefficients);
        proj.filter(*cloud_projected);

        this->processedCloudUpdate(cloud_projected);

        return;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::pcaTransform()
    {
        Eigen::Matrix3f eigen_vector;
        Eigen::Vector4f mean_vector;
        this->computePCAMatrix(eigen_vector, mean_vector);
        this->pca_transform_matrix = this->computeTransformMatrix(eigen_vector, mean_vector);

        this->processedCloudTransform(this->processed_cloud, this->pca_transform_matrix);

        return;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::scaleTo(float scale)
    {
        Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Zero(); 
        scaleMatrix(0,0) = scale;     
        scaleMatrix(1,1) = scale;    
        scaleMatrix(2,2) = scale;    
        scaleMatrix(3,3) = 1;      

        this->processedCloudTransform(this->processed_cloud, scaleMatrix);
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::transform(Eigen::Matrix4f transform_matrix)
    {
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector3f translation_vector)
    {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.linear() = rotation_matrix;
        transform.translation() = translation_vector;

        Eigen::Matrix4f transform_matrix = transform.matrix();
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::transform(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix)
    {
        Eigen::Matrix4f transform_matrix = this->computeTransformMatrix(rotation_matrix, translation_matrix);
        
        this->processedCloudTransform(this->processed_cloud, transform_matrix);

        return;
    }
    

    template<typename PointT>
    void PointCloud2Proc<PointT>::transformWorldToImg()
    {
        this->processedCloudTransform(this->processed_cloud, this->computeTransformMatrixWorldToImg());
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::transformImgToWorld()
    {
        this->processedCloudTransform(this->processed_cloud, this->computeTransformMatrixImgToWorld());
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::findRectangleCornersInPCAPlane()
    {
        this->pca_points_.clear();
        for (const auto& point : *this->processed_cloud) {
            this->pca_points_.emplace_back(point.x, point.y);
        }

        this->pointcloud_rect_box = cv::minAreaRect(cv::Mat(this->pca_points_));
        this->pointcloud_rect_box.points(this->rect_corners_2d);

        return;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::optimizeRectangleAdjustCentroid(float constraint_width, float constraint_height, float optimize_offset_ratio, float optimize_precision)
    {
        if (this->pointcloud_rect_box.size.area() == 0 || this->pca_points_.empty())
        {
            return;
        }

        // std::cout << "constraint: " << cv::Size2f(constraint_width, constraint_height) << " || pointcloud_rect_box: " << this->pointcloud_rect_box.size << std::endl;
        cv::Size2f newSize(constraint_width, constraint_height);
        if (constraint_width > constraint_height && this->pointcloud_rect_box.size.width < this->pointcloud_rect_box.size.height || 
            constraint_width < constraint_height && this->pointcloud_rect_box.size.width > this->pointcloud_rect_box.size.height)
        {
            newSize = cv::Size2f(constraint_height, constraint_width);
        }

        int minLoss = INT_MAX;
        cv::RotatedRect optimized_rect;

        float x_offset = optimize_offset_ratio * this->pointcloud_rect_box.size.width;
        float y_offset = optimize_offset_ratio * this->pointcloud_rect_box.size.height;
        for (float dx = -x_offset / 2; dx <= x_offset / 2; dx += optimize_precision) 
        {
            for (float dy = -y_offset / 2; dy <= y_offset / 2; dy += optimize_precision) 
            {
                cv::Point2f newCenter(this->pointcloud_rect_box.center.x + dx, this->pointcloud_rect_box.center.y + dy);
                cv::RotatedRect testRect(newCenter, newSize, this->pointcloud_rect_box.angle);
                cv::Point2f testCorners[4];
                testRect.points(testCorners);

                int loss = 0;
                for (const auto& point : this->pca_points_) {
                    if (!isPointInQuad(testCorners, point)) {
                        loss++;
                    }
                }

                if (loss < minLoss) 
                {
                    minLoss = loss;
                    optimized_rect = testRect;
                }
            }
        }

        this->pointcloud_rect_box = optimized_rect;
        this->pointcloud_rect_box.points(this->rect_corners_2d);
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::optimizeRectangleAngleAtCentroid(float constraint_width, float constraint_height, float angle_range_ratio, float angle_precision)
    {
        if (this->pointcloud_rect_box.size.area() == 0 || this->pca_points_.empty())
        {
            return;
        }

        cv::Size2f newSize(constraint_width, constraint_height);
        if ((constraint_width > constraint_height && this->pointcloud_rect_box.size.width < this->pointcloud_rect_box.size.height) ||
            (constraint_width < constraint_height && this->pointcloud_rect_box.size.width > this->pointcloud_rect_box.size.height))
        {
            newSize = cv::Size2f(constraint_height, constraint_width);
        }

        int minLoss = INT_MAX;
        cv::RotatedRect optimized_rect;

        float angle_range = 2 * M_PI * angle_range_ratio;
        float start_angle = this->pointcloud_rect_box.angle - angle_range / 2;
        float end_angle = this->pointcloud_rect_box.angle + angle_range / 2;

        for (float angle = start_angle; angle <= end_angle; angle += angle_precision)
        {
            cv::RotatedRect testRect(this->pointcloud_rect_box.center, newSize, angle);
            cv::Point2f testCorners[4];
            testRect.points(testCorners);

            int loss = 0;
            for (const auto& point : this->pca_points_)
            {
                if (!isPointInQuad(testCorners, point))
                {
                    loss++;
                }
            }

            if (loss < minLoss)
            {
                minLoss = loss;
                optimized_rect = testRect;
            }
        }
        // std::cout << "constraint: " << cv::Size2f(constraint_width, constraint_height) << " || pointcloud_rect_box: " << this->pointcloud_rect_box.size << std::endl;
        this->pointcloud_rect_box = optimized_rect;
        this->pointcloud_rect_box.points(this->rect_corners_2d);
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::transformCornersTo3D()
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
    template<typename PointT>
    void PointCloud2Proc<PointT>::transformCornersTo3D(Eigen::Matrix4f transform_matrix)
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

    template<typename PointT>
    void PointCloud2Proc<PointT>::extractConcaveHull(double alpha)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_hull(new typename pcl::PointCloud<PointT>);

        cloud_hull = this->calculateConcaveHull(this->processed_cloud, alpha);

        this->processedCloudUpdate(cloud_hull);

        return;
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::extractConvexHull()
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_hull(new typename pcl::PointCloud<PointT>);

        cloud_hull = this->calculateConvexHull(this->processed_cloud);

        this->processedCloudUpdate(cloud_hull);

        return;
    }




    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::extractNearestRectangleCorners(bool useStatisticalOutlierFilter, OptimizationMethod optimization_method,
                                                                                        float constraint_width, float constraint_height, 
                                                                                        float optimize_range_ratio, float optimize_precision)
    {
        
        if(this->normalClusterExtraction().size() == 0)
        {
            return this->NOCLOUD();
        }
        if(this->extractNearestClusterCloud().indices.size() == 0)
        {
            return this->NOCLOUD();
        }
        if(useStatisticalOutlierFilter)
        {
            if(this->statisticalOutlierFilter() == 0)
            {
                return this->NOCLOUD();
            }
        }
        if(this->planeSegmentation().indices.size() == 0)
        {
            return this->NOCLOUD();
        }
        // this->planeProjection();
        this->pcaTransform();
        this->findRectangleCornersInPCAPlane();

        if(optimization_method == OptimizationMethod::AdjustCentroid && constraint_width != 0.0 && constraint_height != 0.0)
        {
            this->optimizeRectangleAdjustCentroid(constraint_width, constraint_height, optimize_range_ratio, optimize_precision);
        }
        if(optimization_method == OptimizationMethod::AngleAtCentroid && constraint_width != 0.0 && constraint_height != 0.0)
        {
            this->optimizeRectangleAngleAtCentroid(constraint_width, constraint_height, optimize_range_ratio, optimize_precision);
        }
        
        this->transformCornersTo3D();
        this->processedCloudUpdate(this->raw_cloud, this->computeNearestClusterIndices(this->raw_cloud, this->clusters));

        return this->rect_corners_3d;
        
        
        
    }



    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::getFilterBoxCorners(Eigen::Vector4f min_point, Eigen::Vector4f max_point)
    {
        typename pcl::PointCloud<PointT>::Ptr corners(new typename pcl::PointCloud<PointT>);

        pcl::PointXYZI point;

        point.x = min_point.x(); point.y = min_point.y(); point.z = min_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = max_point.x(); point.y = min_point.y(); point.z = min_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = max_point.x(); point.y = max_point.y(); point.z = min_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = min_point.x(); point.y = max_point.y(); point.z = min_point.z(); point.intensity = 0;
        corners->emplace_back(point);


        point.x = min_point.x(); point.y = min_point.y(); point.z = max_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = max_point.x(); point.y = min_point.y(); point.z = max_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = max_point.x(); point.y = max_point.y(); point.z = max_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        point.x = min_point.x(); point.y = max_point.y(); point.z = max_point.z(); point.intensity = 0;
        corners->emplace_back(point);

        
        return corners;
    }
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::getFilterBoxCorners(Eigen::Vector3f box_center, float length_x, float length_y, float length_z,
                                                                                float angle_x, float angle_y, float angle_z)
    {
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(angle_x * M_PI / 180.0, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(angle_y * M_PI / 180.0, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(angle_z * M_PI / 180.0, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f rotate(q);

        Eigen::Translation3f translate_to_center(box_center);

        Eigen::Affine3f transform = translate_to_center * rotate;

        std::vector<Eigen::Vector3f> corners_at_origin;
        corners_at_origin.push_back(Eigen::Vector3f(-length_x/2, -length_y/2, -length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(length_x/2, -length_y/2, -length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(length_x/2, length_y/2, -length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(-length_x/2, length_y/2, -length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(-length_x/2, -length_y/2, length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(length_x/2, -length_y/2, length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(length_x/2, length_y/2, length_z/2));
        corners_at_origin.push_back(Eigen::Vector3f(-length_x/2, length_y/2, length_z/2));

        typename pcl::PointCloud<PointT>::Ptr corners(new typename pcl::PointCloud<PointT>);
        for (const auto& corner_at_origin : corners_at_origin) 
        {
            Eigen::Vector3f transformed_corner = transform * corner_at_origin; 
            pcl::PointXYZI point;
            point.x = transformed_corner.x(); point.y = transformed_corner.y(); point.z = transformed_corner.z(); point.intensity = 0.0; 
            corners->emplace_back(point);
        }
        return corners;
    }


    template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr PointCloud2Proc<PointT>::computeNormals(int k_search)
    {
        // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(this->tree);
        normal_estimator.setInputCloud(this->processed_cloud);
        normal_estimator.setKSearch(k_search);
        normal_estimator.compute(*this->normals);

        return this->normals;
    }

    template<typename PointT>
    pcl::PointIndices PointCloud2Proc<PointT>::computeNearestClusterIndices(typename pcl::PointCloud<PointT>::Ptr input_cloud, std::vector<pcl::PointIndices> input_clusters, 
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
            typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>);
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

    template<typename PointT>
    void PointCloud2Proc<PointT>::computePCAMatrix(Eigen::Matrix3f& eigen_vector, Eigen::Vector4f& mean_vector)
    {
        pcl::PCA<pcl::PointXYZI> pca;
        pca.setInputCloud(this->processed_cloud);
        eigen_vector = pca.getEigenVectors();
        mean_vector = pca.getMean();
    }

    template<typename PointT>
    Eigen::Matrix4f PointCloud2Proc<PointT>::computeTransformMatrix(Eigen::Matrix3f rotation_matrix, Eigen::Vector4f translation_matrix)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0, 0) = rotation_matrix.transpose();  
        transform.block<3,1>(0, 3) = -1.0 * (rotation_matrix.transpose() * translation_matrix.head<3>()); 
        
        return transform;
    }

    template<typename PointT>
    Eigen::Matrix4f PointCloud2Proc<PointT>::computeTransformMatrixWorldToImg()
    {
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
    template<typename PointT>
    Eigen::Matrix4f PointCloud2Proc<PointT>::computeTransformMatrixImgToWorld()
    {
        float scale = 1 / 1000.0f;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Zero(); 
        transform(0, 2) = 1 * scale;    
        transform(1, 0) = -1 * scale;   
        transform(2, 1) = -1 * scale;   
        transform(3, 3) = 1;        

        return transform;
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::sortPointByNormal(typename pcl::PointCloud<PointT>::Ptr points, const Eigen::Vector3f& normal)
    {
        pcl::PointXYZI center;
        for (const auto& p : points->points) {
            center.x += p.x;
            center.y += p.y;
            center.z += p.z;
        }
        center.x /= points->points.size();
        center.y /= points->points.size();
        center.z /= points->points.size();

        Eigen::Vector3f orthogonal_vector;
        float x_abs = std::abs(normal.x());
        float y_abs = std::abs(normal.y());
        float z_abs = std::abs(normal.z());

        if (x_abs <= y_abs && x_abs <= z_abs) {
            orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
        } else if (y_abs <= x_abs && y_abs <= z_abs) {
            orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
        } else {
            orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
        }

        Eigen::Vector3f ref_vector = normal.cross(orthogonal_vector).normalized(); 
        Eigen::Vector3f plane_vector = normal.cross(ref_vector).normalized(); 

        std::vector<std::pair<float, int>> angle_indices;
        float max_z = -std::numeric_limits<float>::max();
        int max_z_index = 0;
        for (int i = 0; i < points->points.size(); ++i) {
            const auto& p = points->points[i];
            if (p.z > max_z) {
                max_z = p.z;
                max_z_index = i;
            }
            Eigen::Vector3f vec(p.x - center.x, p.y - center.y, p.z - center.z);
            float angle = atan2(vec.dot(ref_vector), vec.dot(plane_vector)); 
            angle_indices.emplace_back(angle, i);
        }

        std::sort(angle_indices.begin(), angle_indices.end());

        while (angle_indices.front().second != max_z_index) {
            std::rotate(angle_indices.begin(), angle_indices.begin() + 1, angle_indices.end());
        }

        typename pcl::PointCloud<PointT> sorted_cloud;
        for (const auto& angle_index : angle_indices) {
            sorted_cloud.push_back(points->points[angle_index.second]);
        }

        *points = sorted_cloud;

    }


    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::calculateConcaveHull(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, double alpha)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_hull(new typename pcl::PointCloud<PointT>);

        pcl::ConcaveHull<pcl::PointXYZI> concave_hull;
        concave_hull.setInputCloud(input_cloud);
        concave_hull.setAlpha(alpha);  
        concave_hull.reconstruct(*cloud_hull);

        return cloud_hull;
    }
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::calculateConvexHull(const typename pcl::PointCloud<PointT>::Ptr &input_cloud)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_hull(new typename pcl::PointCloud<PointT>);
        pcl::ConvexHull<pcl::PointXYZI> convex_hull;
        convex_hull.setInputCloud(input_cloud);
        convex_hull.reconstruct(*cloud_hull);

        return cloud_hull;
    }





    template<typename PointT>
    void PointCloud2Proc<PointT>::rawCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        this->raw_cloud->clear();
        pcl::copyPointCloud(*cloud, *this->raw_cloud);
    }
    template<typename PointT>
    void PointCloud2Proc<PointT>::processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        if(cloud == this->processed_cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr write_in_cloud(new typename pcl::PointCloud<PointT>);
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
    template<typename PointT>
    void PointCloud2Proc<PointT>::processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices cluster_indices)
    {
        if(cloud == this->processed_cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr write_in_cloud(new typename pcl::PointCloud<PointT>);
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
    template<typename PointT>
    void PointCloud2Proc<PointT>::processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices> clusters_indices)
    {
        if(cloud == this->processed_cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr write_in_cloud(new typename pcl::PointCloud<PointT>);
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
    template<typename PointT>
    void PointCloud2Proc<PointT>::processedCloudUpdate(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::Indices indices)
    {
        if(cloud == this->processed_cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr write_in_cloud(new typename pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            this->processed_cloud->clear();
            pcl::copyPointCloud(*write_in_cloud, indices, *this->processed_cloud);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::copyPointCloud(*cloud, indices, *this->processed_cloud);
        }
    }

    template<typename PointT>
    void PointCloud2Proc<PointT>::processedCloudTransform(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Matrix4f transform_matrix)
    {
        if (cloud == this->processed_cloud)
        {
            typename pcl::PointCloud<PointT>::Ptr write_in_cloud(new typename pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*cloud, *write_in_cloud);
            pcl::transformPointCloud(*write_in_cloud, *this->processed_cloud, transform_matrix);
        }
        else
        {
            this->processed_cloud->clear();
            pcl::transformPointCloud(*cloud, *this->processed_cloud, transform_matrix);
        }
    }

    template<typename PointT>
    std::vector<std::string> PointCloud2Proc<PointT>::iterateFilesFromPath(std::string folderPath)
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

    template<typename PointT>
    void PointCloud2Proc<PointT>::removeOriginPoint()
    {
        if(this->remove_origin_point_)
        {
            this->boxFilter(Eigen::Vector4f(-0.0001, -0.0001, -0.0001, 1.0), Eigen::Vector4f(0.0001, 0.0001, 0.0001, 1.0), true);
        }
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PointCloud2Proc<PointT>::NOCLOUD()
    {
        return no_cloud;
    }

}





#endif