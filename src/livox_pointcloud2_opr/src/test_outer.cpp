#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // 拟合平面
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
	
	int main()
	{
		//-----------------------------读取点云----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("src/livox_pointcloud2_opr/pcd/20240509061333.pcd", *cloud) < 0)
	{
		PCL_ERROR("点云读取失败！\n");
		return -1;
	}
	//--------------------------RANSAC拟合平面--------------------------
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	
	ransac.setDistanceThreshold(0.01);	//设置距离阈值，与平面距离小于0.1的点作为内点
	ransac.computeModel();				//执行模型估计
	//-------------------------根据索引提取内点--------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> inliers;				//存储内点索引的容器
	ransac.getInliers(inliers);			//提取内点索引
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
	//----------------------------输出模型参数---------------------------
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);
	cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
		<< coefficient[3] << " = 0" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
 
    
        std::cerr << "Cloud before projection: " << std::endl;
        for (size_t i = 0; i < cloud->points.size(); ++i)
            std::cerr << "    " << cloud->points[i].x << " "
            << cloud->points[i].y << " "
            << cloud->points[i].z << std::endl;
    
        // Create a set of planar coefficients with X=Y=0,Z=1
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = coefficient[0];
        coefficients->values[1] = coefficient[1];
        coefficients->values[2] = coefficient[2];
        coefficients->values[3] = coefficient[3];
    
        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);
    
        std::cerr << "Cloud after projection: " << std::endl;
        for (size_t i = 0; i < cloud_projected->points.size(); ++i)
            std::cerr << "    " << cloud_projected->points[i].x << " "
            << cloud_projected->points[i].y << " "
            << cloud_projected->points[i].z << std::endl;

		pcl::visualization::CloudViewer viewer("viewer");
		viewer.showCloud(cloud);
		sleep(3);
		viewer.showCloud(cloud_projected);
		sleep(30);
		
		return 0;
	}
// ————————————————

//                             版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
// 原文链接：https://blog.csdn.net/McQueen_LT/article/details/118102088