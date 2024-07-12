#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_to_rviz_publisher");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/quintinumi/data/PCD/scans12.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file sample_rgb.pcd \n");
        return (-1);
    }

    // 创建旋转矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float theta = 0 * M_PI / 180.0; // 顺时针转 23.5 度
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

    // 应用旋转
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(0.05);
    while (ros::ok()) {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}