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

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h"
#include <pcl/visualization/cloud_viewer.h> 

	
int main()
{

	livox_hikcamera_cal::pointcloud2_opr::PointCloud2Proc<pcl::PointXYZI> pc_process;
	std::vector<std::string> file_paths;
	file_paths = pc_process.iterateFilesFromPath("src/livox_hikcamera_cal/pcd");

	for(const auto file_path:file_paths)
	{
		pc_process.loadPointCloudFile(file_path);
		
		// Detect caliboard corners
		pcl::PointCloud<pcl::PointXYZI>::Ptr corners;
		corners = pc_process.extractNearestRectangleCorners(true);

		// Alternative
		// pc_process.normalClusterExtraction();
		// pc_process.extractNearestClusterCloud();
		// pc_process.planeSegmentation();
		// pc_process.planeProjection();
		// pc_process.pcaTransform();
		// pc_process.findRectangleCornersInPCAPlane();
		// pc_process.transformCornersTo3D();
		// pc_process.processedCloudUpdate(this->raw_cloud, this->computeNearestClusterIndices(this->raw_cloud, this->clusters));

		// Visualized
		pcl::visualization::PCLVisualizer viewer_final("Point Cloud Viewer");
		viewer_final.setBackgroundColor(0, 0, 0);
		

		if(corners->size() != 0)
		{
			//Return corners
			pcl::PointCloud<pcl::PointXYZI>::Ptr original_corners(pc_process.get3DRectCorners());

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red_color(original_corners, 255, 0, 0);
			viewer_final.addPointCloud(original_corners, red_color, "corners");
			viewer_final.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "corners");

			viewer_final.addLine((*original_corners)[0], (*original_corners)[1], "line1");
			viewer_final.addLine((*original_corners)[1], (*original_corners)[2], "line2");
			viewer_final.addLine((*original_corners)[2], (*original_corners)[3], "line3");
			viewer_final.addLine((*original_corners)[3], (*original_corners)[0], "line4");
		}
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud(pc_process.getRawPointcloud());
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rawCloud_color(rawCloud, 255, 255, 255);
		viewer_final.addPointCloud(rawCloud, rawCloud_color, "raw_cloud");
		viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(pc_process.getProcessedPointcloud());
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_plane_color(cloud_plane, 0, 255, 0);
		viewer_final.addPointCloud(cloud_plane, cloud_plane_color, "cloud_plane");
		viewer_final.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_plane");


	
		while (!viewer_final.wasStopped()) {
			viewer_final.spinOnce(100);
		}
	}
	
	return 0;
}
