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

#include "livox_hikcamera_cal/corners_subscriber_publisher.h"
#include "livox_hikcamera_cal/calibration_tool.h"

using namespace livox_hikcamera_cal;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pointcloud_process_node");
	ros::NodeHandle rosHandle;


	std::string frame_id;
	std::string topic_pc_sub;
    std::string topic_pc_pub;
	std::string topic_corners_sub;
    std::string topic_corners_pub;
	rosHandle.param("frame_id", frame_id, std::string("livox_frame"));
	rosHandle.param("pointcloud_process_pc_sub_topic", topic_pc_sub, std::string("/livox/lidar"));
    rosHandle.param("pointcloud_process_pc_pub_topic", topic_pc_pub, std::string("/livox/lidar_proc"));
	rosHandle.param("pointcloud_process_corners_sub_topic", topic_corners_sub, std::string("/livox_hikcamera_cal/calibration_corners"));
    rosHandle.param("pointcloud_process_corners_pub_topic", topic_corners_pub, std::string("/livox_hikcamera_cal/pointcloud_corners"));


	float caliboard_width;
	float caliboard_height;
	rosHandle.param("caliboard_width", caliboard_width, 800.0f);
	rosHandle.param("caliboard_height", caliboard_height, 600.0f);
	caliboard_width /= 1000;
	caliboard_height /= 1000;


	pointcloud2_opr::PointCloudSubscriberPublisher pointcloud_SUB_PUB(rosHandle, topic_pc_sub, topic_pc_pub);		
	pointcloud2_opr::PointCloud2Proc pc_process(true); // Remove Origin Point Published by livox_ros_driver2
	PointcloudFilterReconfigure filterRecfg;
    RQTConfig rqtCfg;
	RvizDrawing rviz_drawing;

	CornersPublisherSubscriber corners_SUB_PUB(rosHandle, frame_id, topic_corners_sub, topic_corners_pub);


	ros::Rate rate(30);

	while(ros::ok())
	{	
		ros::spinOnce();

		if(pointcloud_SUB_PUB.getPointcloudXYZI()->size() == 0 || !pointcloud_SUB_PUB.getPointcloudXYZI())
		{
			// ROS_INFO("Waiting For Point Cloud Subscribe\n");
			continue;
		}
		pc_process.setCloud(pointcloud_SUB_PUB.getPointcloudXYZI());
		// ROS_INFO("%ld\n", pointcloud_SUB_PUB.getPointcloudXYZI()->size());
		rqtCfg.TransformFilterConfig = filterRecfg.getTransformConfigure();
		float center_x = rqtCfg.TransformFilterConfig.center_x;
		float center_y = rqtCfg.TransformFilterConfig.center_y;
		float center_z = rqtCfg.TransformFilterConfig.center_z;
		float length_x = rqtCfg.TransformFilterConfig.length_x;
		float length_y = rqtCfg.TransformFilterConfig.length_y;
		float length_z = rqtCfg.TransformFilterConfig.length_z;
		float rotate_x = rqtCfg.TransformFilterConfig.rotate_x;
		float rotate_y = rqtCfg.TransformFilterConfig.rotate_y;
		float rotate_z = rqtCfg.TransformFilterConfig.rotate_z;
		pc_process.boxFilter(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);

		pointcloud_SUB_PUB.publish(pc_process.getProcessedPointcloud());

		if(filterRecfg.isUpdated())
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr box_corners(new pcl::PointCloud<pcl::PointXYZI>);
			box_corners = pc_process.getFilterBoxCorners(Eigen::Vector3f(center_x, center_y, center_z), length_x, length_y, length_z, rotate_x, rotate_y, rotate_z);
			
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
		corners = pc_process.extractNearestRectangleCorners(false, true, caliboard_width, caliboard_height, 0.05);
		CalTool::sortPointByNormalWorldFrame(corners, pc_process.getPlaneNormals());

		std::vector<geometry_msgs::Point> ros_corners;
		for (const auto& corner : *corners) 
		{
			geometry_msgs::Point ros_point;
			ros_point.x = corner.x;
			ros_point.y = corner.y;
			ros_point.z = corner.z;
			ros_corners.push_back(ros_point);
    	}

		std_msgs::Header header;
        corners_SUB_PUB.publish(ros_corners, corners_SUB_PUB.nowHeader());
		

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

			rviz_drawing.addText("corner_1", ros_corners.at(0), "1", 0.3, 0.0, 1.0, 0.0);
			rviz_drawing.addText("corner_2", ros_corners.at(1), "2", 0.3, 0.0, 1.0, 0.0);
			rviz_drawing.addText("corner_3", ros_corners.at(2), "3", 0.3, 0.0, 1.0, 0.0);
			rviz_drawing.addText("corner_4", ros_corners.at(3), "4", 0.3, 0.0, 1.0, 0.0);

			Eigen::Vector3f plane_normals = pc_process.getPlaneNormals().normalized();
			rviz_drawing.addArrow("plane_normals", 
									(ros_corners[0].x + ros_corners[1].x + ros_corners[2].x + ros_corners[3].x) / 4,
									(ros_corners[0].y + ros_corners[1].y + ros_corners[2].y + ros_corners[3].y) / 4,
									(ros_corners[0].z + ros_corners[1].z + ros_corners[2].z + ros_corners[3].z) / 4,
									plane_normals, 0.03, 0.06, 0.06, 0.0, 1.0, 0.0);

			ros_corners.push_back(ros_corners[0]);
			rviz_drawing.addLines("rect_lines", ros_corners, 4, 0.01, 0.0, 1.0, 0.0);
		}

		rviz_drawing.publish();

		rate.sleep();
	}
	
	return 0;
}
