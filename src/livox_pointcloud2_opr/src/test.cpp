#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "pc2_sub_pub.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pc2_sub_pub_test");
    ros::NodeHandle rosHandle;

    livox_pc2_opr::PointCloud2_SubPub pc(&rosHandle);
    std::cout << "height: " << pc.receivedPointCloud->height << " | width:" << pc.receivedPointCloud->width << std::endl;

    return 0;
}