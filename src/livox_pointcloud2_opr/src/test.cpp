#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <livox_pointcloud2_opr/PointcloudFilterConfig.h>

#include "livox_pc2_opr/point_cloud_subscriber_publisher.h"
#include "livox_pc2_opr/point_cloud_process.h"
#include "livox_pc2_opr/dynamic_reconfigure.h"
#include "livox_pc2_opr/recorder.h"
#include "livox_pc2_opr/pcd_saver.h"


void KeyInput_CallBack(std_msgs::Int8::ConstPtr key_ascii, livox_pc2_opr::PointCloudSubscriberPublisher pcSP){

    livox_pc2_opr::PCDSaver pcdsaver("src/livox_pointcloud2_opr/pcd");
    if(key_ascii->data == 10){
        // std::cout << key_ascii->data << std::endl;
        // ROS_INFO("Test------------------------------------------------------------------------");
        std::cout << pcSP.get_pointcloud() << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud = pcSP.get_pointcloud();
        pcdsaver.save(pclCloud);
    }
    // printf("test-----------------------------------\n");
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pc2_sub_pub_test");
    ros::NodeHandle rosHandle;
    
    livox_pc2_opr::PointCloudSubscriberPublisher pcSP(rosHandle, std::string("/livox/lidar_proc"), std::string("/livox/lidar_proc"));

    livox_pc2_opr::PointCloud2Proc pcProc;

    livox_pc2_opr::PointcloudFilterReconfigure filterRecfg;
    livox_pc2_opr::RQTConfig rqtCfg;

    int loopRate = 20;
    ros::Rate loop_rate(loopRate);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pubCloud(new pcl::PointCloud<pcl::PointXYZI>);


    // livox_pc2_opr::Recorder rcd(std::string("src/livox_pointcloud2_opr/bag"), std::string("/livox/lidar"));

    // rcd.start_recording();

    

    ros::Subscriber keySub = rosHandle.subscribe<std_msgs::Int8>("/msg_hikcamera/key_input", 10,
                                                                boost::bind(&KeyInput_CallBack, _1, pcSP));
    

    ros::spin();

    // printf("%d\n", boost::iequals(std::string(".BAD"), std::string(".bag")));
    // std::cout << std::string("ss.BAD").substr(std::string("ss.BAD").length() - 4) << std::endl;
    // ros::Time now = ros::Time::now();
    // // std::stringstream timeStream;
    // // timeStream  << (&now);
    // // std::cout << timeStream.str() << std::endl;
    // const std::time_t time_c = now.sec; 
    // std::tm* tm = std::localtime(&time_c); 
    // std::stringstream ss;
    // ss << std::put_time(tm, "%Y%m%d%H%M%S");
    // std::cout << ss.str() << std::endl;

    // livox_pc2_opr::Recorder recorder();

    // while(ros::ok()){
    
    //     ros::spinOnce();
    // }
    // std::cout << "height: " << pc.receivedPointCloud->height << " | width:" << pc.receivedPointCloud->width << std::endl;

    return 0;
}