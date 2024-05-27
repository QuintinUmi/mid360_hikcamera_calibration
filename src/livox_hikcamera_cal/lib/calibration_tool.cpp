#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/calibration_tool.h"

using namespace livox_hikcamera_cal;


CalTool::CalTool()
{

}
CalTool::~CalTool()
{

}





void CalTool::sortPointByNormalWorldFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal, 
                                bool negetive, const Eigen::Vector3f& ref_point)
{
    if(points->empty())
    {
        return;
    }

    Eigen::Vector3f normal_;
    Eigen::Vector3f center_point(0.0, 0.0, 0.0);

    for(const auto& point:*points)
    {
        center_point[0] += point.x;
        center_point[1] += point.y;
        center_point[2] += point.z;
    }
    center_point /= points->size();

    float dotProduct = normal.dot(center_point - ref_point);

    if (dotProduct < 0) 
    {
        normal_ = -normal;
    }
    else
    {
        normal_ = normal;
    }

    if(negetive)
    {
        normal_ = -normal_;
    }

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
    float x_abs = std::abs(normal_.x());
    float y_abs = std::abs(normal_.y());
    float z_abs = std::abs(normal_.z());

    if (x_abs <= y_abs && x_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
    } else if (y_abs <= x_abs && y_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
    } else {
        orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
    }

    Eigen::Vector3f ref_vector = normal_.cross(orthogonal_vector).normalized(); 
    Eigen::Vector3f plane_vector = normal_.cross(ref_vector).normalized(); 

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

    pcl::PointCloud<pcl::PointXYZI> sorted_cloud;
    for (const auto& angle_index : angle_indices) {
        sorted_cloud.push_back(points->points[angle_index.second]);
    }

    *points = sorted_cloud;

}
void CalTool::sortPointByNormalWorldFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                bool negetive, const Eigen::Vector3f& ref_point)
{
    Eigen::Vector3f normal_;
    Eigen::Vector3f center_point(0.0, 0.0, 0.0);

    for(const auto& point:points)
    {
        center_point[0] += point.x;
        center_point[1] += point.y;
        center_point[2] += point.z;
    }
    center_point /= points.size();

    float dotProduct = normal.dot(center_point - ref_point);

    if (dotProduct < 0) 
    {
        normal_ = -normal;
    }
    else
    {
        normal_ = normal;
    }

    if(negetive)
    {
        normal_ = -normal_;
    }

    cv::Point3f center;
    for (const auto& p : points) {
        center.x += p.x;
        center.y += p.y;
        center.z += p.z;
    }
    center.x /= points.size();
    center.y /= points.size();
    center.z /= points.size();

    Eigen::Vector3f orthogonal_vector;
    float x_abs = std::abs(normal_.x());
    float y_abs = std::abs(normal_.y());
    float z_abs = std::abs(normal_.z());

    if (x_abs <= y_abs && x_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
    } else if (y_abs <= x_abs && y_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
    } else {
        orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
    }

    Eigen::Vector3f ref_vector = normal_.cross(orthogonal_vector).normalized(); 
    Eigen::Vector3f plane_vector = normal_.cross(ref_vector).normalized(); 

    std::vector<std::pair<float, int>> angle_indices;
    float max_z = -std::numeric_limits<float>::max();
    int max_z_index = 0;
    for (int i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        if (p.y < max_z) {
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

    std::vector<cv::Point3f> sorted_points;
    for (const auto& angle_index : angle_indices) {
        sorted_points.push_back(points[angle_index.second]);
    }

    points = sorted_points;

}
void CalTool::sortPointByNormalImgFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal, 
                                bool negetive, const Eigen::Vector3f& ref_point)
{
    if(points->empty())
    {
        return;
    }

    Eigen::Vector3f normal_;
    Eigen::Vector3f center_point(0.0, 0.0, 0.0);

    for(const auto& point:*points)
    {
        center_point[0] += point.x;
        center_point[1] += point.y;
        center_point[2] += point.z;
    }
    center_point /= points->size();

    float dotProduct = normal.dot(center_point - ref_point);

    if (dotProduct < 0) 
    {
        normal_ = -normal;
    }
    else
    {
        normal_ = normal;
    }

    if(negetive)
    {
        normal_ = -normal_;
    }

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
    float x_abs = std::abs(normal_.x());
    float y_abs = std::abs(normal_.y());
    float z_abs = std::abs(normal_.z());

    if (z_abs <= x_abs && z_abs <= y_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
    } else if (y_abs <= x_abs && y_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
    } else {
        orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
    }

    Eigen::Vector3f ref_vector = normal_.cross(orthogonal_vector).normalized(); 
    Eigen::Vector3f plane_vector = normal_.cross(ref_vector).normalized(); 

    std::vector<std::pair<float, int>> angle_indices;
    float min_y = std::numeric_limits<float>::max();
    int min_y_index = 0;
    for (int i = 0; i < points->size(); ++i) {
        const auto& p = points->points[i];
        if (p.y < min_y) {
            min_y = p.y;
            min_y_index = i;
        }
        Eigen::Vector3f vec(p.x - center.x, p.y - center.y, p.z - center.z);
        float angle = atan2(vec.dot(ref_vector), vec.dot(plane_vector)); 
        angle_indices.emplace_back(angle, i);
    }

    std::sort(angle_indices.begin(), angle_indices.end());

    while (angle_indices.front().second != min_y_index) {
        std::rotate(angle_indices.begin(), angle_indices.begin() + 1, angle_indices.end());
    }

    pcl::PointCloud<pcl::PointXYZI> sorted_cloud;
    for (const auto& angle_index : angle_indices) {
        sorted_cloud.push_back(points->points[angle_index.second]);
    }

    *points = sorted_cloud;

}
void CalTool::sortPointByNormalImgFrame(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
                                bool negetive, const Eigen::Vector3f& ref_point)
{
    Eigen::Vector3f normal_;
    Eigen::Vector3f center_point(0.0, 0.0, 0.0);

    for(const auto& point:points)
    {
        center_point[0] += point.x;
        center_point[1] += point.y;
        center_point[2] += point.z;
    }
    center_point /= points.size();

    float dotProduct = normal.dot(center_point - ref_point);

    if (dotProduct < 0) 
    {
        normal_ = -normal;
    }
    else
    {
        normal_ = normal;
    }

    if(negetive)
    {
        normal_ = -normal_;
    }

    cv::Point3f center;
    for (const auto& p : points) {
        center.x += p.x;
        center.y += p.y;
        center.z += p.z;
    }
    center.x /= points.size();
    center.y /= points.size();
    center.z /= points.size();

    Eigen::Vector3f orthogonal_vector;
    float x_abs = std::abs(normal_.x());
    float y_abs = std::abs(normal_.y());
    float z_abs = std::abs(normal_.z());

    if (z_abs <= x_abs && z_abs <= y_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
    } else if (y_abs <= x_abs && y_abs <= z_abs) {
        orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
    } else {
        orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
    }

    Eigen::Vector3f ref_vector = normal_.cross(orthogonal_vector).normalized(); 
    Eigen::Vector3f plane_vector = normal_.cross(ref_vector).normalized(); 

    std::vector<std::pair<float, int>> angle_indices;
    float min_y = std::numeric_limits<float>::max();
    int min_y_index = 0;
    for (int i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        if (p.y < min_y) {
            min_y = p.y;
            min_y_index = i;
        }
        Eigen::Vector3f vec(p.x - center.x, p.y - center.y, p.z - center.z);
        float angle = atan2(vec.dot(ref_vector), vec.dot(plane_vector)); 
        angle_indices.emplace_back(angle, i);
    }

    std::sort(angle_indices.begin(), angle_indices.end());

    while (angle_indices.front().second != min_y_index) {
        std::rotate(angle_indices.begin(), angle_indices.begin() + 1, angle_indices.end());
    }

    std::vector<cv::Point3f> sorted_points;
    for (const auto& angle_index : angle_indices) {
        sorted_points.push_back(points[angle_index.second]);
    }

    points = sorted_points;

}


Eigen::Quaterniond CalTool::averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (const auto& q : quaternions) {
        Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
        M += q_vec * q_vec.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_decomposition(M);
    auto eigen_vectors = eigen_decomposition.eigenvectors();
    auto eigen_values = eigen_decomposition.eigenvalues();

    int max_index = 0;
    double max_eigen_values = eigen_values(0);
    for(int i = 1; i <= 4; i++)
    {
        if(eigen_values(i) > max_eigen_values) 
        {
            max_eigen_values = eigen_values(i);
            max_index = i;
        }
    }

    Eigen::Vector4d max_eigen_vector = eigen_vectors.col(max_index);

    return Eigen::Quaterniond(max_eigen_vector(0), max_eigen_vector(1), max_eigen_vector(2), max_eigen_vector(3)).normalized();
}






void CalTool::transformPointsImgToWorld(std::vector<geometry_msgs::Point32>& points)
{
    float scale =  1 / 1000.0f;

    for(auto& point:points)
    {
        geometry_msgs::Point32 world_point;
        world_point.x = point.z * scale;
        world_point.y = -point.x * scale;
        world_point.z = -point.y * scale;
        point = world_point;
    }
}
void CalTool::transformPointsWorldToImg(std::vector<geometry_msgs::Point32>& points)
{
    float scale = 1000.0f;

    for(auto& point:points)
    {
        geometry_msgs::Point32 image_point;
        image_point.x = -point.y * scale;
        image_point.y = -point.z * scale;
        image_point.z = point.x * scale;
        point = image_point;
    }
}


Eigen::Vector3f CalTool::computeCentroid(const std::vector<geometry_msgs::Point32>& points) 
{
    Eigen::Vector3f centroid(0, 0, 0);
    for(const auto& p : points) 
    {
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= points.size();
    return centroid;
}

void CalTool::alignPointsToCentroid(const std::vector<geometry_msgs::Point32>& points, const Eigen::Vector3f& centroid, Eigen::MatrixXf& points_matrix) 
{
    points_matrix.resize(3, points.size());
    for (size_t i = 0; i < points.size(); ++i) 
    {
        points_matrix.col(i) = Eigen::Vector3f(points[i].x, points[i].y, points[i].z) - centroid;
    }
}

Eigen::Matrix3f CalTool::findRotationByICP(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y) 
{
    Eigen::Matrix3f S = X * Y.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
        U.col(2) *= -1;  
    }

    return V * U.transpose();
}

Eigen::Vector3f CalTool::findTranslation(const Eigen::Vector3f& centroidX, const Eigen::Vector3f& centroidY, const Eigen::Matrix3f& R) 
{
    return centroidY - R * centroidX;
}





int CalTool::SolveSVD(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point_list, vector<cv::Point3f> image_points_list,
                        Eigen::Matrix3d &R_output, Eigen::Vector3d &t_output)
{
    if (pointcloud_point_list->points.size() != image_points_list.size()) {
        return -1; 
    }


    Eigen::Vector3d centroid_pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_img = Eigen::Vector3d::Zero();
    int n = pointcloud_point_list->points.size();


    for (int i = 0; i < n; ++i) {
        centroid_pc += Eigen::Vector3d(pointcloud_point_list->points[i].x, pointcloud_point_list->points[i].y, pointcloud_point_list->points[i].z);
        centroid_img += Eigen::Vector3d(image_points_list[i].x, image_points_list[i].y, image_points_list[i].z);
    }
    centroid_pc /= n;
    centroid_img /= n;


    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d pc(pointcloud_point_list->points[i].x, pointcloud_point_list->points[i].y, pointcloud_point_list->points[i].z);
        Eigen::Vector3d img(image_points_list[i].x, image_points_list[i].y, image_points_list[i].z);
        S += (pc - centroid_pc) * (img - centroid_img).transpose();
    }


    Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    if (U.determinant() * V.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }


    Eigen::Vector3d t = centroid_img - R * centroid_pc;

    R_output = R;
    t_output = t;

    return 0; 

}


float CalTool::pointToLineDistance(const pcl::PointXYZI& point, const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) 
{
    Eigen::Vector2f p(point.x, point.y);
    Eigen::Vector2f pa(a.x, a.y);
    Eigen::Vector2f ba(b.x - a.x, b.y - a.y);
    Eigen::Vector2f pa_p = p - pa;
    float t = pa_p.dot(ba) / ba.dot(ba);
    t = std::max(0.0f, std::min(1.0f, t));
    Eigen::Vector2f closest = pa + t * ba;
    return (p - closest).norm();
}
float CalTool::triangleArea(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3) 
{
    return 0.5 * std::abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
}
float CalTool::quadrilateralArea(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3, const geometry_msgs::Point32& p4) 
{
    float area1 = triangleArea(p1, p2, p3);
    float area2 = triangleArea(p3, p4, p1);

    return area1 + area2;
}

void CalTool::removeBoundingBoxOutliers(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::vector<geometry_msgs::Point32>& corners) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pca_trans_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix3f eigen_vector;
    Eigen::Vector4f mean_vector;

    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    eigen_vector = pca.getEigenVectors();
    mean_vector = pca.getMean();


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0, 0) = eigen_vector.transpose();  
    transform.block<3,1>(0, 3) = -1.0 * (eigen_vector.transpose() * mean_vector.head<3>()); 

    pcl::transformPointCloud(*cloud, *pca_trans_cloud, transform);

    std::vector<cv::Point2f> pca_points;
    for (const auto& point : *pca_trans_cloud) 
    {
        pca_points.emplace_back(point.x, point.y);
    }
    cv::RotatedRect pointcloud_rect_box = cv::minAreaRect(cv::Mat(pca_points));
    cv::Point2f rect_corners_2d[4];
    pointcloud_rect_box.points(rect_corners_2d);

    std::vector<geometry_msgs::Point32> rect_corners_3d;
    for (int i = 0; i < 4; i++) 
    {
        Eigen::Vector4f pt_2d(rect_corners_2d[i].x, rect_corners_2d[i].y, 0, 1);
        Eigen::Vector4f pt_3d = transform.inverse() * pt_2d;
        geometry_msgs::Point32 outer_corners;
        outer_corners.x = pt_3d[0];
        outer_corners.y = pt_3d[1];
        outer_corners.z = pt_3d[2];
        rect_corners_3d.push_back(outer_corners);
    }

    float del_area = std::abs(quadrilateralArea(rect_corners_3d[0], rect_corners_3d[1], rect_corners_3d[2], rect_corners_3d[3]) - 
                                quadrilateralArea(corners[0], corners[1], corners[2], corners[3]));
    float threshold = std::sqrt(del_area) / 16;
    ROS_INFO("threshold = %f\n", threshold);



    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);


    for (size_t i = 0; i < cloud->points.size(); ++i) {
        float min_distance = std::numeric_limits<float>::max();
        for (size_t j = 0; j < corners.size(); ++j) {
            size_t next = (j + 1) % corners.size();  
            float distance = pointToLineDistance(cloud->points[i], corners[j], corners[next]);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        
        if (min_distance <= threshold) {
            inliers->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filtered_cloud);

    pcl::copyPointCloud(*filtered_cloud, *cloud);
}






void CalTool::computeReprojectionErrorsInPixels(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr objectPoints,
    const std::vector<geometry_msgs::Point32>& imageCorners,
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& t,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double& meanError,  
    double& stdDev)     
{
    std::vector<cv::Point3f> corners3D;
    for (const auto& corner : imageCorners) {
        corners3D.emplace_back(cv::Point3f(corner.x * 1000, corner.y * 1000, corner.z * 1000));
    }

    std::vector<cv::Point2f> projectedCorners;
    cv::projectPoints(corners3D, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), cameraMatrix, distCoeffs, projectedCorners);

    std::vector<Line> lines;
    for (size_t i = 0; i < projectedCorners.size(); ++i) {
        cv::Point2f start = projectedCorners[i];
        cv::Point2f end = projectedCorners[(i + 1) % projectedCorners.size()];
        lines.emplace_back(Line{start, end});
    }

    std::vector<double> errors;
    for (const auto& point : *objectPoints) {
        Eigen::Vector3f point3D(point.x * 1000, point.y * 1000, point.z * 1000);
        Eigen::Vector3f cameraPoint = R * point3D + t * 1000;

        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point3f> points = {{cameraPoint.x(), cameraPoint.y(), cameraPoint.z()}};
        cv::projectPoints(points, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), cameraMatrix, distCoeffs, imagePoints);

        double minDistance = std::numeric_limits<double>::max();
        for (const auto& line : lines) {
            double distance = line.distanceToPoint(imagePoints[0]);
            minDistance = std::min(minDistance, distance);
        }

        errors.push_back(minDistance);
    }

    meanError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

    double sumSq = std::accumulate(errors.begin(), errors.end(), 0.0, [&](double acc, double e) {
        return acc + (e - meanError) * (e - meanError);
    });
    stdDev = std::sqrt(sumSq / errors.size());
}
void CalTool::computeReprojectionErrorsInPixels(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr objectPoints,
    const std::vector<std::vector<geometry_msgs::Point32>>& imageCorners_list,
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& t,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double& meanError,  
    double& stdDev)     
{
    size_t minSize = std::min(objectPoints->size(), imageCorners_list.size());
    if(minSize == 0) return;
    std::vector<geometry_msgs::Point32> last_corners;
    std::vector<Line> lines;
    std::vector<double> errors;
    for(int p_index = 0; p_index < minSize; p_index ++) {

        std::vector<geometry_msgs::Point32> imageCorners = imageCorners_list[p_index];

        if (last_corners != imageCorners) {

            std::vector<cv::Point3f> corners3D;
            for (const auto& corner : imageCorners) {
                corners3D.emplace_back(cv::Point3f(corner.x * 1000, corner.y * 1000, corner.z * 1000));
            }
            lines.clear();
            std::vector<cv::Point2f> projectedCorners;
            cv::projectPoints(corners3D, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), cameraMatrix, distCoeffs, projectedCorners);

            
            for (size_t i = 0; i < projectedCorners.size(); ++i) {
                cv::Point2f start = projectedCorners[i];
                cv::Point2f end = projectedCorners[(i + 1) % projectedCorners.size()];
                lines.emplace_back(Line{start, end});
            }
        }
        
        for (const auto& point : *objectPoints) {
            Eigen::Vector3f point3D(point.x * 1000, point.y * 1000, point.z * 1000);
            Eigen::Vector3f cameraPoint = R * point3D + t * 1000;

            std::vector<cv::Point2f> imagePoints;
            std::vector<cv::Point3f> points = {{cameraPoint.x(), cameraPoint.y(), cameraPoint.z()}};
            cv::projectPoints(points, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), cameraMatrix, distCoeffs, imagePoints);

            double minDistance = std::numeric_limits<double>::max();
            for (const auto& line : lines) {
                double distance = line.distanceToPoint(imagePoints[0]);
                minDistance = std::min(minDistance, distance);
            }

            errors.push_back(minDistance);
        }

    }

    meanError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

    double sumSq = std::accumulate(errors.begin(), errors.end(), 0.0, [&](double acc, double e) {
    return acc + (e - meanError) * (e - meanError);
    });
    stdDev = std::sqrt(sumSq / errors.size());
    
}