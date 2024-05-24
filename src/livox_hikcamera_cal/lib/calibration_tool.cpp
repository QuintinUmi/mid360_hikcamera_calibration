#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "livox_hikcamera_cal/calibration_tool.h"

using namespace livox_hikcamera_cal;


CalTool::CalTool()
{

}
CalTool::~CalTool()
{

}

void CalTool::sortPointByNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr points, const Eigen::Vector3f& normal, 
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
// void CalTool::sortPointByNormal(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
//                                 bool negetive, const Eigen::Vector3f& ref_point)
// {
//     Eigen::Vector3f normal_;
//     Eigen::Vector3f center_point(0.0, 0.0, 0.0);

//     for(const auto& point:points)
//     {
//         center_point[0] += point.x;
//         center_point[1] += point.y;
//         center_point[2] += point.z;
//     }
//     center_point /= points.size();

//     float dotProduct = normal.dot(center_point - ref_point);

//     if (dotProduct < 0) 
//     {
//         normal_ = -normal;
//     }
//     else
//     {
//         normal_ = normal;
//     }

//     if(negetive)
//     {
//         normal_ = -normal_;
//     }

//     cv::Point3f center;
//     for (const auto& p : points) {
//         center.x += p.x;
//         center.y += p.y;
//         center.z += p.z;
//     }
//     center.x /= points.size();
//     center.y /= points.size();
//     center.z /= points.size();

//     Eigen::Vector3f orthogonal_vector;
//     float x_abs = std::abs(normal_.x());
//     float y_abs = std::abs(normal_.y());
//     float z_abs = std::abs(normal_.z());

//     if (x_abs <= y_abs && x_abs <= z_abs) {
//         orthogonal_vector = Eigen::Vector3f(1.0, 0.0, 0.0);  
//     } else if (y_abs <= x_abs && y_abs <= z_abs) {
//         orthogonal_vector = Eigen::Vector3f(0.0, 1.0, 0.0);  
//     } else {
//         orthogonal_vector = Eigen::Vector3f(0.0, 0.0, 1.0);  
//     }

//     Eigen::Vector3f ref_vector = normal_.cross(orthogonal_vector).normalized(); 
//     Eigen::Vector3f plane_vector = normal_.cross(ref_vector).normalized(); 

//     std::vector<std::pair<float, int>> angle_indices;
//     float min_z = -std::numeric_limits<float>::max();
//     int min_z_index = 0;
//     for (int i = 0; i < points.size(); ++i) {
//         const auto& p = points[i];
//         if (p.y < min_z) {
//             min_z = p.z;
//             min_z_index = i;
//         }
//         Eigen::Vector3f vec(p.x - center.x, p.y - center.y, p.z - center.z);
//         float angle = atan2(vec.dot(ref_vector), vec.dot(plane_vector)); 
//         angle_indices.emplace_back(angle, i);
//     }

//     std::sort(angle_indices.begin(), angle_indices.end());

//     while (angle_indices.front().second != min_z_index) {
//         std::rotate(angle_indices.begin(), angle_indices.begin() + 1, angle_indices.end());
//     }

//     std::vector<cv::Point3f> sorted_points;
//     for (const auto& angle_index : angle_indices) {
//         sorted_points.push_back(points[angle_index.second]);
//     }

//     points = sorted_points;

// }
void CalTool::sortPointByNormal(std::vector<cv::Point3f>& points, const Eigen::Vector3f& normal, 
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