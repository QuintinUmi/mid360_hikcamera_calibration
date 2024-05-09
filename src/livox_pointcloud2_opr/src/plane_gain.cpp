#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // 拟合平面
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
	//-----------------------------读取点云----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("src/livox_pointcloud2_opr/pcd/20240509061333.pcd", *cloud) < 0)
	{
		PCL_ERROR("点云读取失败！\n");
		return -1;
	}
	//--------------------------RANSAC拟合平面--------------------------
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	
	ransac.setDistanceThreshold(0.01);	//设置距离阈值，与平面距离小于0.1的点作为内点
	ransac.computeModel();				//执行模型估计
	//-------------------------根据索引提取内点--------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	vector<int> inliers;				//存储内点索引的容器
	ransac.getInliers(inliers);			//提取内点索引
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
	//----------------------------输出模型参数---------------------------
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);
	cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
		<< coefficient[3] << " = 0" << endl;
	//-----------------------------结果可视化----------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");													
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");	

	viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");												
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "plane");	
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");	

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}