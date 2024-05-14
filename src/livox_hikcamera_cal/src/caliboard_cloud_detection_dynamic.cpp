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

#include <pcl/visualization/cloud_viewer.h> 

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h"
#include "livox_hikcamera_cal/dynamic_reconfigure.h"
#include "livox_hikcamera_cal/rviz_drawing.h"

using namespace livox_hikcamera_cal;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "caliboard_cloud_detection_dynamic");
	ros::NodeHandle rosHandle;

	pointcloud2_opr::PointCloudSubscriberPublisher pointcloud_SUB_PUB(rosHandle, std::string("/livox/lidar"), std::string("/livox/lidar_proc"));		
	pointcloud2_opr::PointCloud2Proc pc_process;
	PointcloudFilterReconfigure filterRecfg;
    RQTConfig rqtCfg;
	RvizDrawing rviz_drawing;


	ros::Rate rate(30);

	while(ros::ok())
	{	
		ros::spinOnce();

		if(pointcloud_SUB_PUB.getPointcloudXYZI()->size() == 0 || !pointcloud_SUB_PUB.getPointcloudXYZI())
		{
			ROS_INFO("Waiting For Point Cloud Subscribe\n");
			continue;
		}
		pc_process.setCloud(pointcloud_SUB_PUB.getPointcloudXYZI());
		pc_process.boxFilter(Eigen::Vector4f(-0.001, -0.001, -0.001, 1.0), Eigen::Vector4f(0.001, 0.001, 0.001, 1.0), true);
		// ROS_INFO("%ld\n", pointcloud_SUB_PUB.getPointcloudXYZI()->size());
		rqtCfg.FilterConfig = filterRecfg.getConfigure();
		float x_max = rqtCfg.FilterConfig.x_max;
		float y_max = rqtCfg.FilterConfig.y_max;
		float z_max = rqtCfg.FilterConfig.z_max;
		float x_min = rqtCfg.FilterConfig.x_min;
		float y_min = rqtCfg.FilterConfig.y_min;
		float z_min = rqtCfg.FilterConfig.z_min;
		pc_process.boxFilter(Eigen::Vector4f(x_min, y_min, z_min, 1.0), Eigen::Vector4f(x_max, y_max, z_max, 1.0));

		pointcloud_SUB_PUB.publish(pc_process.getProcessedPointcloud());

		// Detect caliboard corners
		pcl::PointCloud<pcl::PointXYZI>::Ptr corners;
		// corners = pc_process.extractNearestRectangleCorners(true, 50, 1.5);
		corners = pc_process.extractNearestRectangleCorners(false);
		// if(pc_process.normalClusterExtraction(0.05235988, 0.1F, 90, 30, 200, 250000).size() == 0)
        // {
        //     continue;
        // }
        // if(pc_process.extractNearestClusterCloud().indices.size() == 0)
        // {
        //     continue;
        // }
        // // if(useStatisticalOutlierFilter)
        // // {
        // //     if(this->statisticalOutlierFilter() == 0)
        // //     {
        // //         return this->NOCLOUD();
        // //     }
        // // }
        // if(pc_process.planeSegmentation().indices.size() == 0)
        // {
        //     continue;
        // }
        // // this->planeProjection();
        // pc_process.pcaTransform();
        // pc_process.findRectangleCornersInPCAPlane();
        // pc_process.transformCornersTo3D();
		// corners = pc_process.get3DRectCorners();


		std::vector<geometry_msgs::Point> ros_corners;
		for (const auto& corner : *corners) 
		{
			geometry_msgs::Point ros_point;
			ros_point.x = corner.x;
			ros_point.y = corner.y;
			ros_point.z = corner.z;
			ros_corners.push_back(ros_point);
    	}

		

		if(corners->size() == 0)
		{
			// rviz_drawing.deleteAllObject();
			rviz_drawing.deleteObject("corners");
			rviz_drawing.deleteObject("rect_lines");
		}
		else
		{
			rviz_drawing.addPoints("corners", ros_corners, 0.03, 1.0, 0.0, 0.0);
			ros_corners.push_back(ros_corners[0]);
			rviz_drawing.addLines("rect_lines", ros_corners, 4, 0.01, 0.0, 1.0, 0.0);
		}

		rviz_drawing.publish();

		rate.sleep();
	}
	
	return 0;
}
