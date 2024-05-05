#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class pcl_sub
{
private:
  ros::NodeHandle n;
  ros::Subscriber subCloud;
  ros::Publisher pubCloud;
  sensor_msgs::PointCloud2 msg;  //接收到的点云消息
  sensor_msgs::PointCloud2 adjust_msg;  //等待发送的点云消息
  pcl::PointCloud<pcl::PointXYZI> adjust_pcl;   //建立了一个pcl的点云，作为中间过程

public:
  pcl_sub():
    n("~"){
    subCloud = n.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &pcl_sub::getcloud, this); //接收点云数据，进入回调函数getcloud()
    pubCloud = n.advertise<sensor_msgs::PointCloud2>("/adjustd_cloud", 1000);  //建立了一个发布器，主题是/adjusted_cloud，方便之后发布调整后的点云
  }

  //回调函数
  void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr adjust_pcl_ptr (new pcl::PointCloud<pcl::PointXYZI>);   //放在这里是因为，每次都需要重新初始化
    pcl::fromROSMsg(*laserCloudMsg, *adjust_pcl_ptr);  //把msg消息转化为点云
    adjust_pcl = *adjust_pcl_ptr;  
    // for (int i = 0; i < adjust_pcl.points.size(); i++)
    // //把接收到的点云位置不变，颜色全部变为绿色
    // {
    //   adjust_pcl.points[i].r = 0;
    //   adjust_pcl.points[i].g = 255;
    //   adjust_pcl.points[i].b = 0;
    // }
    pcl::toROSMsg(adjust_pcl, adjust_msg);  //将点云转化为消息才能发布
    pubCloud.publish(adjust_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
  }

  ~pcl_sub(){}

};

int main(int argc, char** argv) {

  ros::init(argc, argv, "colored");  //初始化了一个节点，名字为colored

  pcl_sub ps;

  ros::spin();
  return 0;
}
