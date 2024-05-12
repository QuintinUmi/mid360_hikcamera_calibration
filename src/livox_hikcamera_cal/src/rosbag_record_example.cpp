#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "livox_hikcamera_cal/recorder.h"

using namespace livox_hikcamera_cal;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosbag_record_example");
    ros::NodeHandle rosHandle;

    Recorder::Topic lidar_msg("/livox/lidar", Recorder::MessageType::PointCloud2);
    Recorder::Topic camera_msg("/hikcamera/img_stream", Recorder::MessageType::Image);
    Recorder rcd(std::string("src/livox_hikcamera_cal/bag"));
    rcd.addTopic(lidar_msg);
    rcd.addTopic(camera_msg);

    ROS_INFO("Start Recording\n");
    rcd.startRecording();
    
    ros::spin();
    ROS_INFO("Stop Recording\n");
    return 0;
}