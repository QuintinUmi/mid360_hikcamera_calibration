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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

#include "livox_pc2_opr/point_cloud_process.h"
	
	int main()
	{
		//-----------------------------读取点云----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("src/livox_pointcloud2_opr/pcd/20240509061323.pcd", *rawCloud) < 0)
	{
		PCL_ERROR("点云读取失败！\n");
		return -1;
	}
	//------------------------------------------------------------------------------------------
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// 	// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
// 	// sor.setInputCloud (rawCloud);
// 	// sor.setMeanK (50);
// 	// sor.setStddevMulThresh (0.5);
// 	// sor.filter (*cloud);

// 	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
// 	// tree->setInputCloud (rawCloud);//　桌子平面上其他的点云
// 	// std::vector<pcl::PointIndices> cluster_indices;// 点云团索引
// 	// pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
// 	// ec.setClusterTolerance (0.1);                    // 设置近邻搜索的搜索半径为2cm
// 	// ec.setMinClusterSize (200);                       // 设置一个聚类需要的最少的点数目为100
// 	// ec.setMaxClusterSize (25000);                     // 设置一个聚类需要的最大点数目为25000
// 	// ec.setSearchMethod (tree);                        // 设置点云的搜索机制
// 	// ec.setInputCloud (rawCloud);
// 	// ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
// 	// pcl::copyPointCloud<pcl::PointXYZ>(*rawCloud, cluster_indices[cluster_indices.size()-1], *cloud);


// 	//-------------------------------------------------------------------------------------------------------------

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
   //求法线
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (rawCloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute (*normals);
//    //直通滤波在Z轴的0到1米之间
// //   pcl::IndicesPtr indices (new std::vector <int>);
// //   pcl::PassThrough<pcl::PointXYZ> pass;
// //   pass.setInputCloud (cloud);
// //   pass.setFilterFieldName ("z");
// //   pass.setFilterLimits (0.0, 1.0);
// //   pass.filter (*indices);
  //聚类对象<点，法线>
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (100);  //最小的聚类的点数
  reg.setMaxClusterSize (25000);  //最大的
  reg.setSearchMethod (tree);    //搜索方式
  reg.setNumberOfNeighbours (30);    //设置搜索的邻域点的个数
  reg.setInputCloud (rawCloud);         //输入点
  //reg.setIndices (indices);
  reg.setInputNormals (normals);     //输入的法线
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);  //设置平滑度
  reg.setCurvatureThreshold (0.1);     //设置曲率的阀值

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  

// //-------------------------------------------------------------------------------------
Eigen::Vector4f referencePoint(0.0, 0.0, 0.0, 0.0); // 观察点位置

// // 读取点云数据

float minDistance = std::numeric_limits<float>::max();
int nearestPlaneIndex = -1;

for (size_t i = 0; i < clusters.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*rawCloud, clusters[i], *cluster);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    float distance = (centroid.head<3>() - referencePoint.head<3>()).norm(); // 计算距离
    if (distance < minDistance) {
        minDistance = distance;
        nearestPlaneIndex = i;
    }
}


// //--------------------------------------------------------------------------------------


  pcl::PointCloud<pcl::PointXYZ>::Ptr normalFilter_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*rawCloud, clusters[nearestPlaneIndex], *normalFilter_cloud);

// //   std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
// //   std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
// //   std::cout << "These are the indices of the points of the initial" <<
// //     std::endl << "cloud that belong to the first cluster:" << std::endl;
 
// // //  int counter = 0;
// // //   while (counter < clusters[0].indices.size ())
// // //   {
// // //     std::cout << clusters[0].indices[counter] << ", ";
// // //     counter++;
// // //     if (counter % 10 == 0)
// // //       std::cout << std::endl;
// // //   }
// // //   std::cout << std::endl;
  
// //   //可视化聚类的结果
// // //   pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
// // //   pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
// // //   cluster_viewer.showCloud(colored_cloud);

// //   pcl::copyPointCloud(*rawCloud, clusters[0], *cloud);

// // //   pcl::visualization::CloudViewer viewer("viewer");
  

// // 	sleep(5);

// //--------------------------------------------------------------------------------------------------
	// 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true); // 优化系数
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);        // 增加迭代次数提高拟合机会
    seg.setDistanceThreshold(0.01);    // 设置距离阈值

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(normalFilter_cloud);
    seg.segment(*inliers, *coefficients);

	// 提取内点
	extract.setInputCloud(normalFilter_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_plane);
// 	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;


	

// 	// //--------------------------RANSAC拟合平面--------------------------
// 	// pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(rawCloud));
// 	// pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	
// 	// ransac.setDistanceThreshold(0.1);	//设置距离阈值，与平面距离小于0.1的点作为内点
// 	// ransac.computeModel();				//执行模型估计
// 	//-------------------------根据索引提取内点--------------------------
// 	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
// 	// std::vector<int> inliers;				//存储内点索引的容器
// 	// ransac.getInliers(inliers);			//提取内点索引
// 	// Eigen::VectorXf coefficient;
// 	// ransac.getModelCoefficients(coefficient);
// 	// cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
// 	// 	<< coefficient[3] << " = 0" << endl;


// 	//----------------------------输出模型参数---------------------------
	
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
 
    
//         std::cerr << "Cloud before projection: " << std::endl;
//         for (size_t i = 0; i < cloud->points.size(); ++i)
//             std::cerr << "    " << cloud->points[i].x << " "
//             << cloud->points[i].y << " "
//             << cloud->points[i].z << std::endl;
    
//         // Create a set of planar coefficients with X=Y=0,Z=1
//         // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//         // coefficients->values.resize(4);
//         // coefficients->values[0] = coefficient[0];
//         // coefficients->values[1] = coefficient[1];
//         // coefficients->values[2] = coefficient[2];
//         // coefficients->values[3] = coefficient[3];
    
        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_plane);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);
    
//         std::cerr << "Cloud after projection: " << std::endl;
//         for (size_t i = 0; i < cloud_projected->points.size(); ++i)
//             std::cerr << "    " << cloud_projected->points[i].x << " "
//             << cloud_projected->points[i].y << " "
//             << cloud_projected->points[i].z << std::endl;

// 		// pcl::visualization::CloudViewer viewer("viewer");
// 		// viewer.showCloud(cloud_plane);
// 		// sleep(5);
// 		// viewer.showCloud(cloud_projected);
// 		// sleep(5);

// /**************************************************************************/

// 		// pcl::VoxelGrid<pcl::PointXYZ> sor;
// 		// sor.setInputCloud(cloud_projected);
// 		// sor.setLeafSize(0.05f, 0.05f, 0.05f);
// 		// sor.filter(*cloud_projected);

// //---------------------------------------------------------------------------------------------------------

    // 使用PCA计算点云的主方向
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud_projected);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector4f mean_vector = pca.getMean();

    // 计算旋转矩阵以对齐点云到坐标轴
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0, 0) = eigen_vectors.transpose();  // 旋转
    transform.block<3,1>(0, 3) = -1.0 * (eigen_vectors.transpose() * mean_vector.head<3>());  // 平移

	// 计算逆变换矩阵
    Eigen::Matrix4f inverse_transform = transform.inverse();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr rawTransformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_projected, *transformed_cloud, transform);


// 	// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
// 	// sor.setInputCloud (rawTransformed_cloud);
// 	// sor.setMeanK (50);
// 	// sor.setStddevMulThresh (0.5);
// 	// sor.filter (*transformed_cloud);


//     // // 计算边界框
//     // pcl::PointXYZ min_pt, max_pt;
//     // pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);



//     // // 输出角点
//     // std::cout << "Rectangle corners:" << std::endl;
//     // std::cout << "Corner 1: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
//     // std::cout << "Corner 2: (" << min_pt.x << ", " << max_pt.y << ", " << min_pt.z << ")" << std::endl;
//     // std::cout << "Corner 3: (" << max_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
//     // std::cout << "Corner 4: (" << max_pt.x << ", " << max_pt.y << ", " << min_pt.z << ")" << std::endl;

// 	// // 定义角点
//     // std::vector<pcl::PointXYZ> corners = {
//     //     pcl::PointXYZ(min_pt.x, min_pt.y, min_pt.z),
//     //     pcl::PointXYZ(min_pt.x, max_pt.y, min_pt.z),
//     //     pcl::PointXYZ(max_pt.x, min_pt.y, min_pt.z),
//     //     pcl::PointXYZ(max_pt.x, max_pt.y, min_pt.z)
//     // };

	

	// // 变换角点回原始坐标系
    // pcl::PointCloud<pcl::PointXYZ>::Ptr original_corners(new pcl::PointCloud<pcl::PointXYZ>);

    // // 变换角点回原始坐标系
    // for (auto &corner : corners) {
    //     Eigen::Vector4f corner_point(corner.x, corner.y, corner.z, 1.0);
    //     corner_point = inverse_transform * corner_point;
	// 	original_corners->push_back(pcl::PointXYZ(corner_point[0], corner_point[1], corner_point[2]));
    //     std::cout << "Transformed Corner: (" << corner_point[0] << ", " << corner_point[1] << ", " << corner_point[2] << ")" << std::endl;
    // }

	std::vector<cv::Point2f> points;
    for (const auto& point : *transformed_cloud) {
        points.emplace_back(point.x, point.y);
    }

	// 计算最小面积矩形
    cv::RotatedRect box = cv::minAreaRect(cv::Mat(points));
	cv::Point2f rect_points[4];
    box.points(rect_points);

	// 反向变换到原始的三维空间
    // Eigen::Matrix4f inv_proj_matrix = inverse_transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_corners(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < 4; i++) {
        Eigen::Vector4f pt_2d(rect_points[i].x, rect_points[i].y, 0, 1);
        Eigen::Vector4f pt_3d = inverse_transform * pt_2d;
        original_corners->push_back(pcl::PointXYZ(pt_3d[0], pt_3d[1], pt_3d[2]));
    }


//-------------------------------------------------------------------------------------------------

    livox_pc2_opr::PointCloud2Proc pc_process;
    pc_process.loadPointCloudFile("src/livox_pointcloud2_opr/pcd/20240509061323.pcd");
    // pc_process.extractNearestRectangleCorners();

    

//--------------------------------------------------------------------------------------------------


	// 可视化
    pcl::visualization::PCLVisualizer viewer_final("Point Cloud Viewer");
    viewer_final.setBackgroundColor(0, 0, 0);


    // pcl::PointCloud<pcl::PointXYZ>::Ptr original_corners(pc_process.get3DRectCorners());

    // 添加角点并以红色显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(original_corners, 255, 0, 0);
    viewer_final.addPointCloud(original_corners, red_color, "corners");
    viewer_final.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "corners");

	
    // pc_process.resetCloud();
    // pc_process.normalClusterExtraction();
    // pc_process.extractNearestCluster();
    // pc_process.planeSegmentation();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(pc_process.getRawPointcloud());
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rawCloud_color(rawCloud, 255, 255, 255);
	viewer_final.addPointCloud(rawCloud, rawCloud_color, "raw_cloud");
	viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");
    
    // pc_process.transform(pc_process.getPCATransformMatrix().inverse());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(pc_process.getProcessedPointcloud());
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_color(cloud_plane, 0, 255, 0);
    viewer_final.addPointCloud(cloud_plane, cloud_plane_color, "cloud_plane");
	viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_plane");
	// viewer_final.addPointCloud(cloud_projected, "cloud");

    

    // 连接角点形成矩形
    viewer_final.addLine((*original_corners)[0], (*original_corners)[1], "line1");
    viewer_final.addLine((*original_corners)[1], (*original_corners)[2], "line2");
    viewer_final.addLine((*original_corners)[2], (*original_corners)[3], "line3");
    viewer_final.addLine((*original_corners)[3], (*original_corners)[0], "line4");

    // 让可视化界面保持开启状态
    while (!viewer_final.wasStopped()) {
        viewer_final.spinOnce(100);
    }

		
		return 0;
	}
// ————————————————

//                             版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
// 原文链接：https://blog.csdn.net/McQueen_LT/article/details/118102088