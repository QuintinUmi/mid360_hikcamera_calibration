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
using namespace livox_hikcamera_cal::pointcloud2_opr;



void PC2SubCallBack(const sensor_msgs::PointCloud2ConstPtr &rcvCloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*rcvCloud, *tempCloud);

	ROS_INFO("msg_timestamp: %u.%u, pcl_timestamp: %lu, point_size: %09ld\n", rcvCloud->header.stamp.sec, rcvCloud->header.stamp.nsec, tempCloud->header.stamp, tempCloud->points.size());


}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "caliboard_cloud_detection_dynamic");
	ros::NodeHandle rosHandle;


	ros::Subscriber pointcloud2_SUB = rosHandle.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, PC2SubCallBack);


	ros::Rate rate(30);

	while(ros::ok())
	{	
		ros::spinOnce();


		rate.sleep();
	}
	
	return 0;
}
