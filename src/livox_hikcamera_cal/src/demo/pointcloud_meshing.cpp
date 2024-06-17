#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

int main() {

    auto start_time = std::chrono::high_resolution_clock::now();
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/quintinumi/r3live_output/scans2.pcd", *cloud);

    // 创建用于存储法线的点云
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 估计法线
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_xyzrgb(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree_xyzrgb);
    ne.setRadiusSearch(0.01);  // 设置法线估计的搜索半径
    ne.compute(*normals);

    // 合并点云和法线
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // 为新的点云创建搜索树
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    tree->setInputCloud(cloud_with_normals);

    // 初始化GreedyProjectionTriangulation
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(300);
    gp3.setMaximumSurfaceAngle(M_PI / 6);  // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(true);

    // 设置输入为包含法线的点云和对应的搜索树
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);

    pcl::PolygonMesh triangles;
    gp3.reconstruct(triangles);

    // 保存网格
    pcl::io::saveVTKFile("/home/quintinumi/r3live_output/mesh.vtk", triangles);

    auto end_time = std::chrono::high_resolution_clock::now();
    printf("Working Time: %ld", std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());

    return 0;
}