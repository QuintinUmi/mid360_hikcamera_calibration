#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "point_cloud_subscriber_publisher.h"
#include "point_cloud_process.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pc2_sub_pub_test");
    ros::NodeHandle rosHandle;
    
    livox_pc2_opr::PointCloudSubscriberPublisher pcSP(rosHandle, std::string("/livox/lidar"), std::string("/livox/lidar_proc"));

    livox_pc2_opr::PointCloud2Proc pcProc;

    int loopRate = 20;
    ros::Rate loop_rate(loopRate);
    
    // ros::spin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pubCloud(new pcl::PointCloud<pcl::PointXYZI>);

    while(ros::ok()){
    
        ros::spinOnce();

        // std::cout << pcSP.get_pointcloud() << std::endl;
        if(!pcSP.get_pointcloud() || pcSP.get_pointcloud()->points.size() == 0){
            // std::cout << pcSP.get_pointcloud()->points.size() << std::endl;
            printf("Waiting for Pointcloud Received\n");
            loop_rate.sleep();
            continue;
        }
        
        pcProc.setCloud(pcSP.get_pointcloud());
        pcProc.filter(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), Eigen::Vector4f(100.0f, 100.0f, 100.0f, 1.0f));
        // std::cout << pcSP.get_pointcloud()->header.frame_id << std::endl;
        pcSP.publish(pcProc.get_processed_pointcloud());
        
        // pubCloud->points.emplace_back(pcSP.get_pointcloud()->points[0]);
        // pubCloud->points.emplace_back(pcSP.get_pointcloud()->points[1]);
        // pubCloud->points.emplace_back(pcSP.get_pointcloud()->points[2]);
        // pubCloud->header.frame_id = pcSP.get_pointcloud()->header.frame_id;
        // pubCloud->header.stamp = pcSP.get_pointcloud()->header.stamp;
        // pcSP.publish(pubCloud);
        // pubCloud->points.pop_back();
        // pubCloud->points.pop_back();
        // pubCloud->points.pop_back();

        loop_rate.sleep();
    }
    // std::cout << "height: " << pc.receivedPointCloud->height << " | width:" << pc.receivedPointCloud->width << std::endl;

    return 0;
}