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
    Recorder::Topic camera_msg("/hikcamera/img_stream/compressed", Recorder::MessageType::Image);
    Recorder rcd(std::string("/home/quintinumi/project/ws_mid360_hikcamera_calibration/src/bag"));
    rcd.addTopic(lidar_msg);
    rcd.addTopic(camera_msg);

    int record_frame_rate = 30;
    ros::Rate frame_rate(record_frame_rate);

    ROS_INFO("Start Recording\n");
    rcd.startRecording();
    while(ros::ok())
    {
        ros::spinOnce();
        frame_rate.sleep();
    }  
    ROS_INFO("Stop Recording\n");
    return 0;
}