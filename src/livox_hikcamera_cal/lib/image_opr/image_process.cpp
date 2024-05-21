#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include "image_transport/image_transport.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <opencv2/core/eigen.hpp> 

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/conversion_bridge.h"

#include "livox_hikcamera_cal/image_opr/image_process.h"


using namespace livox_hikcamera_cal::image_opr;

Eigen::Quaterniond ImageProc::averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions) 
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




