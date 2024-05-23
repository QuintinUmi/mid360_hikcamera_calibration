#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_hikcamera_cal/OrthoFilterConfig.h>

#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h"
#include "livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h"
#include "livox_hikcamera_cal/dynamic_reconfigure.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point_cloud_filter");
    ros::NodeHandle ros_handle;
    
    livox_hikcamera_cal::pointcloud2_opr::PointCloudSubscriberPublisher pointcloud_SUB_PUB(ros_handle, std::string("/livox/lidar"), std::string("/livox/lidar_proc"));

    livox_hikcamera_cal::pointcloud2_opr::PointCloud2Proc processed_pointcloud;

    livox_hikcamera_cal::PointcloudFilterReconfigure box_filter_reconfigure;
    livox_hikcamera_cal::RQTConfig rqtCfg;

    int loopRate = 20;
    ros::Rate loop_rate(loopRate);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pubCloud(new pcl::PointCloud<pcl::PointXYZ>);

    while(ros::ok()){
    
        ros::spinOnce();

        // std::cout << pointcloud_SUB_PUB.getPointcloud() << std::endl;
        if(!pointcloud_SUB_PUB.getPointcloudXYZI() || pointcloud_SUB_PUB.getPointcloudXYZI()->points.size() == 0){
            // std::cout << pointcloud_SUB_PUB.getPointcloud()->points.size() << std::endl;
            printf("Waiting for Pointcloud Received\n");
            loop_rate.sleep();
            continue;
        }

        rqtCfg.OrthoFilterConfig =  box_filter_reconfigure.getOrthoConfigure();

        processed_pointcloud.setCloud(pointcloud_SUB_PUB.getPointcloudXYZI());
        // processed_pointcloud.filter(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), Eigen::Vector4f(100.0f, 100.0f, 100.0f, 1.0f));
        processed_pointcloud.boxFilter(Eigen::Vector4f(rqtCfg.OrthoFilterConfig.x_min, rqtCfg.OrthoFilterConfig.y_min, rqtCfg.OrthoFilterConfig.z_min, 1.0f), 
                    Eigen::Vector4f(rqtCfg.OrthoFilterConfig.x_max, rqtCfg.OrthoFilterConfig.y_max, rqtCfg.OrthoFilterConfig.z_max, 1.0f));
        // std::cout << pointcloud_SUB_PUB.getPointcloud()->header.frame_id << std::endl;
        pointcloud_SUB_PUB.publish(processed_pointcloud.getProcessedPointcloud());

        loop_rate.sleep();
    }
    // std::cout << "height: " << pc.receivedPointCloud->height << " | width:" << pc.receivedPointCloud->width << std::endl;

    return 0;
}