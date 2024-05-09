 
//--------------------------------------------------------------------------------------------
//   source /home/xx/catkin_ws/devel/setup.bash && rosrun my_cam_lidar_calib  cloud 1.pcd
//
//     提取点云中的标定板
//--------------------------------------------------------------------------------------------

#include <iostream>
#include <math.h>
#include <cmath>
#include <string.h>
//常用点云类
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //滤波相关
#include <pcl/visualization/cloud_viewer.h>   //类cloud_viewer头文件申明
//分割类
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
//图像类
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

//标定板长、宽
#define LONG 0.6
#define WIDE 0.67
#define DisThre 0.03//平面分割阈值
#define SLEEP 2     //睡眠时间

using namespace pcl;
using namespace cv;
using namespace std;

float plane_A ,plane_B ,plane_C,plane_D;
void  cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char *zhou,int min,int max);
void  Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input );
void  cloudStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void  output_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane ,pcl::ModelCoefficients::Ptr  coefficients,pcl::PointIndices::Ptr inliers);
bool  choicePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane);

void  cloudEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
bool  Collinear(pcl::PointXYZ A,pcl::PointXYZ B,pcl::PointXYZ C);
void  LineFitLeastFit(const std::vector<cv::Point2f> &_points,float & _k,float & _b,float & _r);
void  intersection(cv::Point2f &point,float & A_k,float & A_b,float & B_k,float & B_b);
void  SpatialPoint(cv::Point2f &point2D,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void  addSpatialPoint(cv::Point2f &point2A,cv::Point2f &point2B,float & B_k,float & B_b,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


pcl::visualization::CloudViewer viewer("viewer");

int main(int argc,char** argv)
{
    // string pcd_mame = argv[1]; 
    string pcd_mame = "src/livox_pointcloud2_opr/pcd/20240509061323.pcd";
    //打开PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(pcd_mame, *cloud) == -1)
    {
        PCL_ERROR("Could not read pcd file!\n");
    
    }
    // pcl::io::loadPCDFile(pcd_mame, *cloud);
    viewer.showCloud(cloud);
    sleep(SLEEP);
    //直通滤波  
    // cloudPassThrough(cloud,"y",-1.8,1.8);
    // cloudPassThrough(cloud,"x",1.8,7);
    // cloudPassThrough(cloud,"z",-2,2);
    cloudPassThrough(cloud,"y",-100,100);
    cloudPassThrough(cloud,"x",-100,100);
    cloudPassThrough(cloud,"z",-100,100);
    viewer.showCloud(cloud);
    sleep(SLEEP);
    //去除离群点
    cloudStatisticalOutlierRemoval( cloud);
    viewer.showCloud(cloud);
    pcl::io::savePCDFileASCII("myRemoval.pcd",*cloud);
    cout<<"save myRemoval.pcd success !!"<<endl;
    sleep(SLEEP);
    //平面提取
    Plane_fitting(cloud);
    // viewer.showCloud(cloud);
    //边界提取 139
    //cloudEdge(cloud);
    // output_plane();
    sleep(200);

    return (0);
}

void Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(300);
    seg.setDistanceThreshold(DisThre);

    while (cloud_input->size() > 100)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud_input);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_input);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);//输出平面

        
        if (cloud_plane->size()>300)
        {
            output_plane(cloud_plane ,coefficients,inliers);
        }
        // 移除plane
        extract.setNegative(true);
        extract.filter(*cloud_p);
        *cloud_input = *cloud_p;
        std::cout << cloud_plane->size() << std::endl;
    }

}

void  output_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane ,pcl::ModelCoefficients::Ptr  coeff,pcl::PointIndices::Ptr inliers)
{
    std::cout << choicePlane(cloud_plane) << std::endl;
   if(choicePlane(cloud_plane))
   {
       plane_A=coeff->values[0];
       plane_B=coeff->values[1];
       plane_C=coeff->values[2];
       plane_D=coeff->values[3];

       cout<< coeff->values[0]<<"  "<<coeff->values[1]<<"  "<<coeff->values[2]<<"  "<<coeff->values[3]<<endl;
       viewer.showCloud(cloud_plane);
       sleep(SLEEP);
       pcl::io::savePCDFileASCII("myplane.pcd",*cloud_plane);
       cout<<"save myplane.pcd success !!"<<endl;

       //边界提取
       cloudEdge(cloud_plane);
   }

}

//选取标定板所在平面
bool  choicePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double maxDistance=sqrt(pow(LONG,2)+pow(WIDE,2));
    double oldDistance=0;
    for(std::size_t i=0; i<cloud->size(); i++)
    {
        double thisDistance=sqrt(pow(cloud->points[i].x-cloud->points[0].x,2)+pow(cloud->points[i].y-cloud->points[0].y,2)+pow(cloud->points[i].z-cloud->points[0].z,2));

        if(oldDistance<thisDistance)
        {
            oldDistance=thisDistance;
            if(oldDistance>maxDistance+0.05)
               return false;
        }
    }
    if(oldDistance<LONG)
        return false;
    return true;
}

//直通滤波器对点云进行处理
void  cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char *zhou,int min,int max)
{
     pcl::PassThrough<pcl::PointXYZ> passthrough;
     passthrough.setInputCloud(cloud);//输入点云
     passthrough.setFilterFieldName(zhou);//对z轴进行操作
     passthrough.setFilterLimits(min,max);//设置直通滤波器操作范围
     passthrough.filter(*cloud);//);//执行滤波

}
//去除离群点
void cloudStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
    sor.setInputCloud (cloud);                           //设置待滤波的点云
    sor.setMeanK (20);                                 //设置在进行统计时考虑的临近点个数
    sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
    sor.filter (*cloud);
}
void  cloudEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int index[16][2];
    memset(index,-1,sizeof(index));//赋值
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(std::size_t i=0 ; i< cloud->size(); i++)
    {
        //计算垂直俯仰角
        float angle = atan(cloud->points[i].z / sqrt( pow(cloud->points[i].x,2) +  pow(cloud->points[i].y,2))) * 180 / M_PI;
        int scanID = 0;
        scanID = int((angle + 15) / 2 + 0.5);// + 0.5 用于四舍五入
        if(0<=scanID && scanID<16)
        {

            if(index[scanID][0]==-1)
                index[scanID][0]=i;
            else
                index[scanID][1]=i;

        }
        pcl::PointXYZRGB thisColor;
        thisColor.x=cloud->points[i].x;
        thisColor.y=cloud->points[i].y;
        thisColor.z=cloud->points[i].z;
        thisColor.r=255;
        thisColor.g=255;
        thisColor.b=255;
        cloud_all->push_back(thisColor);

    }
    //提取边缘
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge_right(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0 ;i<16;i++)
    {
        if(index[i][0]!=-1)
        {
            pcl::PointXYZRGB thisColor;
            thisColor.x=cloud->points[index[i][0]].x;
            thisColor.y=cloud->points[index[i][0]].y;
            thisColor.z=cloud->points[index[i][0]].z;
            thisColor.r=255;
            thisColor.g=255;
            thisColor.b=255;
            //cloud_all->push_back(thisColor);
            cloud_edge->push_back(thisColor);
            cloud_edge_left->push_back(cloud->points[index[i][0]]);
        }
    }
    for(int i=15;i>=0;i--)
    {
        if(index[i][1]!=-1)
        {
            pcl::PointXYZRGB thisColor;
            thisColor.x=cloud->points[index[i][1]].x;
            thisColor.y=cloud->points[index[i][1]].y;
            thisColor.z=cloud->points[index[i][1]].z;
            thisColor.r=255;
            thisColor.g=255;
            thisColor.b=255;
            //cloud_all->push_back(thisColor);
            cloud_edge->push_back(thisColor);
            cloud_edge_right->push_back(cloud->points[index[i][1]]);
        }
    }

    //划分4条线
    int d_a=0;
    int b_c=0;
    vector<cv::Point2f> pointsA;
    vector<cv::Point2f> pointsB;
    vector<cv::Point2f> pointsC;
    vector<cv::Point2f> pointsD;

    for(std::size_t i=0; i<cloud_edge_left->size()-2; i++)
    {
        //左侧第三个点到前两个点的距离
        if(Collinear(cloud_edge_left->points[i],cloud_edge_left->points[i+1],cloud_edge_left->points[i+2]))
        {
            d_a=i+1;
            break;
        }
    }
    for(std::size_t i=0; i<cloud_edge_right->size()-2; i++)
    {
        //右侧第三个点到前两个点的距离
        if(Collinear(cloud_edge_right->points[i],cloud_edge_right->points[i+1],cloud_edge_right->points[i+2]))
        {
            b_c=i+1;
            break;
        }
    }
    for(std::size_t i=0; i<cloud_edge_left->size(); i++)
    {
        if(i<d_a)
        {
            cv::Point2f thisPoint;
            thisPoint.x=cloud_edge_left->points[i].y;
            thisPoint.y=cloud_edge_left->points[i].z;
            pointsD.push_back(thisPoint);
        }
        else
        {
            cv::Point2f thisPoint;
            thisPoint.x=cloud_edge_left->points[i].y;
            thisPoint.y=cloud_edge_left->points[i].z;
            pointsA.push_back(thisPoint);
        }
    }
    for(std::size_t i=0; i<cloud_edge_right->size(); i++)
    {
        if(i<b_c)
        {
            cv::Point2f thisPoint;
            thisPoint.x=cloud_edge_right->points[i].y;
            thisPoint.y=cloud_edge_right->points[i].z;
            pointsB.push_back(thisPoint);
        }
        else
        {
            cv::Point2f thisPoint;
            thisPoint.x=cloud_edge_right->points[i].y;
            thisPoint.y=cloud_edge_right->points[i].z;
            pointsC.push_back(thisPoint);
        }
    }

    float A_k,A_b,A_r;
    float B_k,B_b,B_r;
    float C_k,C_b,C_r;
    float D_k,D_b,D_r;
    LineFitLeastFit(pointsA,A_k,A_b, A_r);
    LineFitLeastFit(pointsB,B_k,B_b, B_r);
    LineFitLeastFit(pointsC,C_k,C_b, C_r);
    LineFitLeastFit(pointsD,D_k,D_b, D_r);
    //求yoz平面交点
    cv::Point2f pointAb;
    cv::Point2f pointBc;
    cv::Point2f pointCd;
    cv::Point2f pointDa;
    intersection(pointAb, A_k, A_b, B_k, B_b);
    intersection(pointBc, B_k, B_b, C_k, C_b);
    intersection(pointCd, C_k, C_b, D_k, D_b);
    intersection(pointDa, D_k, D_b, A_k, A_b);

    //求空间点 
    SpatialPoint(pointAb,cloud_all);
    SpatialPoint(pointBc,cloud_all);
    SpatialPoint(pointCd,cloud_all);
    SpatialPoint(pointDa,cloud_all);

    //增加空间点
    addSpatialPoint(pointAb,pointBc,B_k,B_b,cloud_all);
    addSpatialPoint(pointBc,pointCd,C_k,C_b,cloud_all);
    addSpatialPoint(pointCd,pointDa,D_k,D_b,cloud_all);
    addSpatialPoint(pointDa,pointAb,A_k,A_b,cloud_all);

    // 保存边界点云和拟合点云 
    viewer.showCloud(cloud_edge);
    sleep(SLEEP);
    pcl::io::savePCDFileASCII("myEdge.pcd",*cloud_edge);
    cout<<"save myEdge.pcd success !!"<<endl;

    viewer.showCloud(cloud_all);
    sleep(SLEEP);
    pcl::io::savePCDFileASCII("myEstimate.pcd",*cloud_all);
    cout<<"save myEstimate.pcd success !!"<<endl;
}
//共线判断 (C到AB的距离)
bool  Collinear(pcl::PointXYZ A,pcl::PointXYZ C,pcl::PointXYZ B)
{
    // 三角形面积*2=叉积的模|axb|=a*b*sin(theta)
    float SABC = sqrt(((B.x - A.x)*(B.y - C.y) - (B.x - C.x)*(B.y - A.y)) * ((B.x - A.x)*(B.y - C.y) - (B.x - C.x)*(B.y - A.y))
                    + ((B.x - A.x)*(B.z - C.z) - (B.x - C.x)*(B.z - A.z)) * ((B.x - A.x)*(B.z - C.z) - (B.x - C.x)*(B.z - A.z))
                    + ((B.y - A.y)*(B.z - C.z) - (B.y - C.y)*(B.z - A.z)) * ((B.y - A.y)*(B.z - C.z) - (B.y - C.y)*(B.z - A.z)));
    //底边边长
    float lAC = sqrt((A.x - C.x)*(A.x - C.x) + (A.y - C.y)*(A.y - C.y) + (A.z - C.z)*(A.z - C.z));
    //点到直线的距离
    float ld = SABC / lAC;
    //cout<< "ld="<< ld<< "  ac:"<<lAC<<endl;
    if(ld<0.08)
        return false;
    return true;

}

//二维直线拟合
void LineFitLeastFit(const std::vector<cv::Point2f> &_points,float & _k,float & _b,float & _r)
{
    //https://blog.csdn.net/jjjstephen/article/details/108053148?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_title~default-0.pc_relevant_default&spm=1001.2101.3001.4242.1&utm_relevant_index=3
    float B = 0.0f;
    float A = 0.0f;
    float D = 0.0f;
    float C = 0.0f;

    int N = _points.size();
    for (int i = 0; i < N; i++)
    {
            B += _points[i].x;
            A += _points[i].x * _points[i].x;
            D += _points[i].y;
            C += _points[i].x * _points[i].y;
    }
    if ((N * A - B * B) == 0)
            return;
    _k = (N * C - B * D) / (N * A - B * B);
    _b = (A * D - C * B) / (N * A - B * B);
    //计算相关系数
    float Xmean = B / N;
    float Ymean = D / N;

    float tempX = 0.0f;
    float tempY = 0.0f;
    float rDenominator = 0.0;
    for (int i = 0; i < N; i++)
    {
            tempX += (_points[i].x - Xmean) * (_points[i].x - Xmean);
            tempY += (_points[i].y - Ymean) * (_points[i].y - Ymean);
            rDenominator += (_points[i].x - Xmean) * (_points[i].y - Ymean);
    }

    float SigmaXY = sqrt(tempX) * sqrt(tempY);
    if (SigmaXY == 0)
            return;
    _r = rDenominator / SigmaXY;
    //cout<<"r:"<<_r<<endl;
}

//求平面两线交点
void intersection(cv::Point2f &point ,float & A_k,float & A_b,float & B_k,float & B_b)
{
    Eigen::Matrix<double,2,2> A;
    A(0,0)=1;
    A(0,1)=-A_k;
    A(1,0)=1;
    A(1,1)=-B_k;
    Eigen::Matrix<double,2,1> B;
    B(0,0)=A_b;
    B(1,0)=B_b;

    Eigen::Matrix<double,2,1> xy=A.fullPivHouseholderQr().solve(B);
    point.x=xy(1,0);
    point.y=xy(0,0);
    //cout<<point.x<<"  "<<point.y<<endl;
}

//空间点估计
void  SpatialPoint(cv::Point2f &point2D,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all)
{
    pcl::PointXYZRGB thisColor;
    thisColor.x=-(plane_B*point2D.x+plane_C*point2D.y+plane_D)/plane_A;
    thisColor.y=point2D.x;
    thisColor.z=point2D.y;
    thisColor.r=0;
    thisColor.g=255;
    thisColor.b=0;
    cloud_all->push_back(thisColor);
}

//添加空间直线上的点
void  addSpatialPoint(cv::Point2f &point2A,cv::Point2f &point2B,float & _k,float & _b,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all)
{
    float step=0.01;
    if(point2A.x < point2B.x)
    {
        for(float a=point2A.x ;a<point2B.x ;a=a+step)
        {
            cv::Point2f thisPoint;
            thisPoint.x=a;
            thisPoint.y=_k*a+_b;
            SpatialPoint(thisPoint,cloud_all);
        }

    }
    else{
        for(float a=point2B.x ;a<point2A.x ;a=a+step)
        {
            cv::Point2f thisPoint;
            thisPoint.x=a;
            thisPoint.y=_k*a+_b;
            SpatialPoint(thisPoint,cloud_all);
        }
    }

}

