#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "livox_pc2_opr/recorder.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosbag_record_example");
    ros::NodeHandle rosHandle;


    livox_pc2_opr::Recorder rcd(std::string("src/livox_pointcloud2_opr/bag"), std::string("/livox/lidar"));

    rcd.startRecording();
    
    ros::spin();

    return 0;
}