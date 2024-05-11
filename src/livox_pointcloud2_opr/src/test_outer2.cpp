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

    livox_pc2_opr::PointCloud2Proc pc_process;
    pc_process.loadPointCloudFile("src/livox_pointcloud2_opr/pcd/20240509061349.pcd");
    pc_process.extractNearestRectangleCorners();
    // pc_process.normalClusterExtraction();
    // pc_process.extractNearestClusterCloud();
    // pc_process.planeSegmentation();
    // pc_process.planeProjection();
    // pc_process.pcaTransform();
    // pc_process.findRectangleCornersInPCAPlane();
    // pc_process.transformCornersTo3D();

    

//--------------------------------------------------------------------------------------------------


	// 可视化
    pcl::visualization::PCLVisualizer viewer_final("Point Cloud Viewer");
    viewer_final.setBackgroundColor(0, 0, 0);


    pcl::PointCloud<pcl::PointXYZ>::Ptr original_corners(pc_process.get3DRectCorners());

    // 添加角点并以红色显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(original_corners, 255, 0, 0);
    viewer_final.addPointCloud(original_corners, red_color, "corners");
    viewer_final.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "corners");

    // 连接角点形成矩形
    viewer_final.addLine((*original_corners)[0], (*original_corners)[1], "line1");
    viewer_final.addLine((*original_corners)[1], (*original_corners)[2], "line2");
    viewer_final.addLine((*original_corners)[2], (*original_corners)[3], "line3");
    viewer_final.addLine((*original_corners)[3], (*original_corners)[0], "line4");
	
    // pc_process.resetCloud();
    // pc_process.normalClusterExtraction();
    // pc_process.extractNearestClusterCloud();
    // pc_process.planeSegmentation();

    pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(pc_process.getRawPointcloud());
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rawCloud_color(rawCloud, 255, 255, 255);
	viewer_final.addPointCloud(rawCloud, rawCloud_color, "raw_cloud");
	viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(pc_process.getProcessedPointcloud());
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_color(cloud_plane, 0, 255, 0);
    viewer_final.addPointCloud(cloud_plane, cloud_plane_color, "cloud_plane");
	viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_plane");
	// viewer_final.addPointCloud(cloud_projected, "cloud");

    

    

    // 让可视化界面保持开启状态
    while (!viewer_final.wasStopped()) {
        viewer_final.spinOnce(100);
    }

		
		return 0;
	}
// ————————————————

//                             版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
// 原文链接：https://blog.csdn.net/McQueen_LT/article/details/118102088