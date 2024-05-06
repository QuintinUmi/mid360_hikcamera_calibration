#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_pointcloud2_opr/PointcloudFilterConfig.h>

#include "point_cloud_subscriber_publisher.h"
#include "point_cloud_process.h"
#include "dynamic_reconfigure.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pc2_sub_pub_test");
    ros::NodeHandle rosHandle;
    
    livox_pc2_opr::PointCloudSubscriberPublisher pcSP(rosHandle, std::string("/livox/lidar"), std::string("/livox/lidar_proc"));

    livox_pc2_opr::PointCloud2Proc pcProc;

    livox_pc2_opr::PointcloudFilterReconfigure filterRecfg;
    livox_pc2_opr::RQTConfig rqtCfg;

    int loopRate = 20;
    ros::Rate loop_rate(loopRate);
    
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

        rqtCfg.FilterConfig =  filterRecfg.get_configure();
        
        pcProc.setCloud(pcSP.get_pointcloud());
        // pcProc.filter(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), Eigen::Vector4f(100.0f, 100.0f, 100.0f, 1.0f));
        pcProc.filter(Eigen::Vector4f(rqtCfg.FilterConfig.x_min, rqtCfg.FilterConfig.y_min, rqtCfg.FilterConfig.z_min, 1.0f), 
                    Eigen::Vector4f(rqtCfg.FilterConfig.x_max, rqtCfg.FilterConfig.y_max, rqtCfg.FilterConfig.z_max, 1.0f));
        // std::cout << pcSP.get_pointcloud()->header.frame_id << std::endl;
        pcSP.publish(pcProc.get_processed_pointcloud());

        loop_rate.sleep();
    }
    // std::cout << "height: " << pc.receivedPointCloud->height << " | width:" << pc.receivedPointCloud->width << std::endl;

    return 0;
}