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

#include "livox_hikcamera_cal/calibration_tool.h"

using namespace livox_hikcamera_cal;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "caliboard_cloud_detection_dynamic");
	ros::NodeHandle rosHandle;

	pointcloud2_opr::PointCloudSubscriberPublisher pointcloud_SUB_PUB(rosHandle, std::string("/livox/lidar"), std::string("/livox/lidar_proc"));		
	pointcloud2_opr::PointCloud2Proc pc_process(true); // Remove Origin Point Published by livox_ros_driver2
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

		if(filterRecfg.isUpdated())
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr box_corners(new pcl::PointCloud<pcl::PointXYZI>);
			box_corners = pc_process.getFilterBoxCorners(Eigen::Vector4f(x_min, y_min, z_min, 1.0), Eigen::Vector4f(x_max, y_max, z_max, 1.0));
			
			std::vector<geometry_msgs::Point> ros_box_corners;
			for (const auto& box_corner : *box_corners) 
			{
				geometry_msgs::Point ros_point;
				ros_point.x = box_corner.x;
				ros_point.y = box_corner.y;
				ros_point.z = box_corner.z;
				ros_box_corners.push_back(ros_point);
			}
			rviz_drawing.addPoints("box_corners", ros_box_corners, 0.1, 1.0, 0.0, 0.0);

			std::vector<geometry_msgs::Point> line_corners;
			line_corners.assign(ros_box_corners.begin() + 0, ros_box_corners.begin() + 4);
			line_corners.emplace_back(*(ros_box_corners.begin() + 0));
			rviz_drawing.addLines("box_line_min", line_corners, visualization_msgs::Marker::LINE_STRIP, 0.05, 1.0, 1.0, 0.0);
			line_corners.assign(ros_box_corners.begin() + 4, ros_box_corners.begin() + 8);
			line_corners.emplace_back(*(ros_box_corners.begin() + 4));
			rviz_drawing.addLines("box_line_max", line_corners, visualization_msgs::Marker::LINE_STRIP, 0.05, 1.0, 1.0, 0.0);

			rviz_drawing.addLine("box_line_middle1", ros_box_corners.at(0), ros_box_corners.at(0 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_drawing.addLine("box_line_middle2", ros_box_corners.at(1), ros_box_corners.at(1 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_drawing.addLine("box_line_middle3", ros_box_corners.at(2), ros_box_corners.at(2 + 4), 0.05, 1.0, 1.0, 0.0);
			rviz_drawing.addLine("box_line_middle4", ros_box_corners.at(3), ros_box_corners.at(3 + 4), 0.05, 1.0, 1.0, 0.0);
		}
		

		// Detect caliboard corners
		pcl::PointCloud<pcl::PointXYZI>::Ptr corners;
		// corners = pc_process.extractNearestRectangleCorners(true, 50, 1.5);
		corners = pc_process.extractNearestRectangleCorners(false);
		CalTool::sortPointByNormal(corners, pc_process.getPlaneNormals());
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

			rviz_drawing.deleteObject("corner_1");
			rviz_drawing.deleteObject("corner_2");
			rviz_drawing.deleteObject("corner_3");
			rviz_drawing.deleteObject("corner_4");

			rviz_drawing.deleteObject("plane_normals");

			rviz_drawing.deleteObject("rect_lines");
		}
		else
		{
			rviz_drawing.addPoints("corners", ros_corners, 0.03, 1.0, 0.0, 0.0);

			rviz_drawing.addText("corner_1", ros_corners.at(0), "1", 0.3, 1.0, 0.0, 0.0);
			rviz_drawing.addText("corner_2", ros_corners.at(1), "2", 0.3, 1.0, 0.0, 0.0);
			rviz_drawing.addText("corner_3", ros_corners.at(2), "3", 0.3, 1.0, 0.0, 0.0);
			rviz_drawing.addText("corner_4", ros_corners.at(3), "4", 0.3, 1.0, 0.0, 0.0);

			Eigen::Vector3f plane_normals = pc_process.getPlaneNormals() * 0.3;
			rviz_drawing.addArrow("plane_normals", 
									(ros_corners[0].x + ros_corners[1].x + ros_corners[2].x + ros_corners[3].x) / 4,
									(ros_corners[0].y + ros_corners[1].y + ros_corners[2].y + ros_corners[3].y) / 4,
									(ros_corners[0].z + ros_corners[1].z + ros_corners[2].z + ros_corners[3].z) / 4,
									plane_normals, 0.03, 0.06, 0.06, 0.0, 1.0, 1.0);

			ros_corners.push_back(ros_corners[0]);
			rviz_drawing.addLines("rect_lines", ros_corners, 4, 0.01, 0.0, 1.0, 0.0);
		}

		rviz_drawing.publish();

		rate.sleep();
	}
	
	return 0;
}
