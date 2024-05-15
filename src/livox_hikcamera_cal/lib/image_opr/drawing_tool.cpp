#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>

#include "opencv2/opencv.hpp"  
// #include "apriltag/apriltag.h"      
#include <aruco/aruco.h>  

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "drawing_tool.h"
#include "param_code.h"


#define PI 3.14159265358979324

using namespace std;
using namespace drt;


// Draw3D::Draw3D(){
//     this->scaleX = 1.0;
//     this->scaleY = 1.0;
//     this->scaleZ = 1.0;
// }
Draw3D::Draw3D(float unitLength ,float scaleX, float scaleY, float scaleZ, cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->unitLength = unitLength;
    this->scaleX = scaleX;
    this->scaleY = scaleY;
    this->scaleZ = scaleZ;
    this->setCameraMatrix = cameraMatrix;
    this->setDisCoffes = disCoffes;

}
Draw3D::Draw3D(cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->setCameraMatrix = cameraMatrix;
    this->setDisCoffes = disCoffes;
}

void Draw3D::set_unitlen(float unitLength){
    this->unitLength;
}
void Draw3D::set_scale(float scaleX, float scaleY, float scaleZ)
{
    this->scaleX = scaleX;
    this->scaleY = scaleY;
    this->scaleZ = scaleZ;
}


void Draw3D::write_in(cv::Point3f &dst, float x, float y, float z)
{
    dst.x = x * this->scaleX;
    dst.y = y * this->scaleY;
    dst.z = z * this->scaleZ;
}
void Draw3D::write_in(vector<cv::Point3f> &dst, float x, float y, float z)
{
    dst.emplace_back(x * this->scaleX, y * this->scaleY, z * this->scaleZ);
}

vector<vector<cv::Point3f>> Draw3D::draw_ortho_coordinate_3d(cv::Point3f centerPoint, float density)
{
    vector<vector<cv::Point3f>> coordinate(4);
    float cx = centerPoint.x;
    float cy = centerPoint.y;
    float cz = centerPoint.z;
    this->write_in(coordinate[0], cx, cy, cz);
    int pointNum;
    pointNum = this->unitLength / density;
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[1], cx + i * density, cy, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[2], cx, cy + i * density, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[3], cx, cy, cz + i * density);

    return coordinate;
}
vector<vector<cv::Point3f>> Draw3D::draw_ortho_coordinate_3d(float cx, float cy, float cz, float density)
{
    vector<vector<cv::Point3f>> coordinate(4);
    this->write_in(coordinate[0], cx, cy, cz);
    int pointNum;
    pointNum = this->unitLength / density;
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[1], cx + i * density, cy, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[2], cx, cy + i * density, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[3], cx, cy, cz + i * density);

    return coordinate;
}
void Draw3D::draw_ortho_coordinate_2d(cv::Mat &imgInputOutput, cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                        float cx, float cy, float cz)
{
    vector<cv::Point3f> p3d;
    this->write_in(p3d, cx, cy, cz);
    this->write_in(p3d, cx + this->unitLength, cy, cz);
    this->write_in(p3d, cx, cy + this->unitLength, cz);
    this->write_in(p3d, cx, cy, cz + this->unitLength);

    vector<cv::Point2f> p2d;
    cv::projectPoints(p3d, rvec, tvec, cameraMatrix, disCoffes, p2d);
    
    cv::line(imgInputOutput, p2d[0], p2d[1], cv::Scalar(0, 0, 255), 3);
    cv::line(imgInputOutput, p2d[0], p2d[2], cv::Scalar(0, 255, 0), 3);
    cv::line(imgInputOutput, p2d[0], p2d[3], cv::Scalar(255, 0, 0), 3);

}


void Draw3D::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec)
{
    if(rvec.empty()){
        rvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    if(tvec.empty()){
        tvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}
void Draw3D::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                        float rx, float ry, float rz, float tx, float ty, float tz)
{
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << rx, ry, rz), tvec = (cv::Mat_<float>(3, 1) << tx, ty, tz);
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}


void Draw3D::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec)
{
    double len = pow(surfaceNorVec.x, 2) + pow(surfaceNorVec.y, 2) + pow(surfaceNorVec.z, 2);
    float uX = surfaceNorVec.x / len;
    float uY = surfaceNorVec.y / len;
    float uZ = surfaceNorVec.z / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
            
    }
}
void Draw3D::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ)
{
    cv::Mat surfaceNorVec;
    double len = pow(surNorX, 2) + pow(surNorY, 2) + pow(surNorZ, 2);
    float uX = surNorX / len;
    float uY = surNorY / len;
    float uZ = surNorZ / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}




void Draw3D::setparam_image_perspective_3d(cv::Mat cameraMatrix, cv::Mat disCoffes,
                        cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec, cv::Mat offsetTvec)
{
    this->setCameraMatrix = cameraMatrix;
    this->setDisCoffes = disCoffes;                          
    this->setImgOriPoint = imgOriPoint;
    this->setImgSizeIn3d = imgSizeIn3d;
    this->setOffsetRvec = offsetRvec;
    this->setOffsetTvec = offsetTvec;
}
void Draw3D::paste_image_perspective_3d(cv::Mat &srcImageToPaste, cv::Mat &dstImagePasteOn, bool remove_background_color, bool center_image_axis, cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat cameraMatrix = this->setCameraMatrix;
    cv::Mat disCoffes = this->setDisCoffes;                            
    cv::Point3f imgOriPoint = this->setImgOriPoint;
    cv::Size imgSizeIn3d = this->setImgSizeIn3d;
    cv::Mat offsetRvec = this->setOffsetRvec;
    cv::Mat offsetTvec = this->setOffsetTvec;

    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }
    
    vector<cv::Point3f> srcImagePoints3D;
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    
    cv::Mat srcImage;
    cv::resize(srcImageToPaste, srcImage, cv::Size(srcImageToPaste.size().width, srcImageToPaste.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    // printf("test------------------------------------\n");
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        this->transform_3d_points(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    cv::projectPoints(srcImagePoints3D, rvec, tvec, cameraMatrix, disCoffes, dstImagePoints2D);

    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};

    cv::Mat warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
    cv::Mat _dstImage;
    cv::warpPerspective(srcImage, _dstImage, warpM, dstImagePasteOn.size());
    
    if(remove_background_color)
    {
        // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
        cv::Point mask[] = {    dstImagePoints2D[0],
                                dstImagePoints2D[1],
                                dstImagePoints2D[3],
                                dstImagePoints2D[2]};
        // std::cout << dstImage.empty() << std::endl;
        cv::fillConvexPoly(dstImagePasteOn, mask, 4, cv::Scalar(0, 0, 0));
    }
    
    dstImagePasteOn = dstImagePasteOn + _dstImage;
}
void Draw3D::paste_image_perspective_3d(cv::Mat &srcImageToPaste, cv::Mat &dstImagePasteOn, bool remove_background_color, bool center_image_axis, vector<cv::Mat> rvecs, vector<cv::Mat> tvecs)
{
    if(rvecs.size() != tvecs.size())
    {
        printf("Sizes of rvecs and tvecs are not equal!\n");
        return;
    }
    // if(rvecs.empty() || tvecs.empty())
    // {
    //     printf("No rvecs or tvecs data. Perspective Failed!"\n);
    //     return;
    // }
    cv::Mat cameraMatrix = this->setCameraMatrix;
    cv::Mat disCoffes = this->setDisCoffes;                            
    cv::Point3f imgOriPoint = this->setImgOriPoint;
    cv::Size imgSizeIn3d = this->setImgSizeIn3d;
    cv::Mat offsetRvec = this->setOffsetRvec;
    cv::Mat offsetTvec = this->setOffsetTvec;

    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }
    
    vector<cv::Point3f> srcImagePoints3D;
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    

    // std::cout << srcImageToPaste.size() << std::endl;
    cv::Mat srcImage;
    cv::resize(srcImageToPaste, srcImage, cv::Size(srcImageToPaste.size().width, srcImageToPaste.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    // printf("test------------------------------------\n");
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        this->transform_3d_points(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};
    cv::Mat warpM, _dstImage;
    for(int i = 0; i < rvecs.size(); i++)
    {
        cv::projectPoints(srcImagePoints3D, rvecs[i], tvecs[i], cameraMatrix, disCoffes, dstImagePoints2D);

        warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
        cv::Mat _dstImage;
        cv::warpPerspective(srcImage, _dstImage, warpM, dstImagePasteOn.size());
        
        if(remove_background_color)
        {
            // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
            cv::Point mask[] = {    dstImagePoints2D[0],
                                    dstImagePoints2D[1],
                                    dstImagePoints2D[3],
                                    dstImagePoints2D[2]};
            // std::cout << dstImage.empty() << std::endl;
            cv::fillConvexPoly(dstImagePasteOn, mask, 4, cv::Scalar(0, 0, 0));
        }
        
        dstImagePasteOn = dstImagePasteOn + _dstImage;
    }
    
}
void Draw3D::paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, 
                                    cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                    cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec, cv::Mat offsetTvec)
{
    if(center_image_axis)
    {
        imgOriPoint.x = imgOriPoint.x - imgSizeIn3d.width / 2;
        imgOriPoint.y = imgOriPoint.y - imgSizeIn3d.height / 2;
    }

    vector<cv::Point3f> srcImagePoints3D;
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);
    this->write_in(srcImagePoints3D, imgOriPoint.x + imgSizeIn3d.width, imgOriPoint.y + imgSizeIn3d.height, imgOriPoint.z);

    cv::resize(srcImage, srcImage, cv::Size(srcImage.size().width, srcImage.size().width * (imgSizeIn3d.height / imgSizeIn3d.width)));
    
    if(!(offsetRvec.empty() && offsetTvec.empty()))
    {
        this->transform_3d_points(srcImagePoints3D, srcImagePoints3D, offsetRvec, offsetTvec);
    }

    vector<cv::Point2f> dstImagePoints2D;
    cv::projectPoints(srcImagePoints3D, rvec, tvec, cameraMatrix, disCoffes, dstImagePoints2D);

    vector<cv::Point2f> srcImagePoints2D = {cv::Point2f(0, 0), cv::Point2f(0, srcImage.cols),
                                            cv::Point2f(srcImage.rows, 0), cv::Point2f(srcImage.rows, srcImage.cols)};

    cv::Mat warpM = cv::getPerspectiveTransform(srcImagePoints2D, dstImagePoints2D);
    cv::Mat _dstImage;
    cv::warpPerspective(srcImage, _dstImage, warpM, dstImage.size());
    
    if(remove_background_color)
    {
        // cv::fillConvexPoly(dstImage, dstImagePoints2D, cv::Scalar(0, 0, 0));
        cv::Point mask[] = {    dstImagePoints2D[0],
                                dstImagePoints2D[1],
                                dstImagePoints2D[3],
                                dstImagePoints2D[2]};
        // std::cout << dstImage.empty() << std::endl;
        cv::fillConvexPoly(dstImage, mask, 4, cv::Scalar(0, 0, 0));
    }
    
    dstImage = dstImage + _dstImage;
    
}   

void Draw3D::center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage)
{
    float srcH = srcImage.size().height, srcW = srcImage.size().width;
    float cX = srcW / 2;
    float cY = srcH / 2;
    float scaleX = this->scaleX;
    float scaleY = this->scaleY;

    cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                            cv::Point2f(srcW ,srcH), 
                            cv::Point2f(0 ,srcH)
                         };

    cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                            cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                            cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                         };

    cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
    cv::warpAffine(srcImage, dstImage, warpM, srcImage.size());
}
void Draw3D::center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage, float scaleX, float scaleY, int flags, int borderMode, const cv::Scalar &borderValue)
{
    float srcH = srcImage.size().height, srcW = srcImage.size().width;
    float cX = srcW / 2;
    float cY = srcH / 2;

    cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                            cv::Point2f(srcW ,srcH), 
                            cv::Point2f(0 ,srcH)
                         };

    cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                            cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                            cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                         };

    cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
    cv::warpAffine(srcImage, dstImage, warpM, srcImage.size(), flags, borderMode, borderValue);
}

// cv::Mat Draw3D::cal_2vec_rvec(cv::Point3f vecOri, cv::Point3f vecDst)
// {
//     cv::Mat _vecOri = (cv::Mat_<float>(3, 1) << vecOri.x, vecOri.y, vecOri.z), _vecOri_;
//     cv::Mat _vecDst = (cv::Mat_<float>(3, 1) << vecDst.x, vecDst.y, vecDst.z), _vecDst_;
//     cv::Mat v, vs, vt, ca;

//     cv::normalize(_vecOri, _vecOri_, 0.0, 1.0);
//     cv::normalize(_vecDst, _vecDst_, 0.0, 1.0);

//     vs = _vecDst_ * _vecOri_;
//     cv::normalize(vs, v, 0.0, 1.0);

//     ca = _vecDst_.dot(_vecOri_);
//     vt = v * (1 - ca);

    
//     // rotationMatrix[0, 0] = vt.x * v.x + ca;
//     // rotationMatrix[1, 1] = vt.y * v.y + ca;
//     // rotationMatrix[2, 2] = vt.z * v.z + ca;
//     // vt.x *= v.y;
//     // vt.z *= v.x;
//     // vt.y *= v.z;

//     // rotationMatrix[0, 1] = vt.x - vs.z;
//     // rotationMatrix[0, 2] = vt.z + vs.y;
//     // rotationMatrix[1, 0] = vt.x + vs.z;
//     // rotationMatrix[1, 2] = vt.y - vs.x;
//     // rotationMatrix[2, 0] = vt.z - vs.y;
//     // rotationMatrix[2, 1] = vt.y + vs.x;
//     cv::Mat R = (cv::Mat_<float>(3, 3) <<   vt.at<float>(0) * v.at<float>(0) + ca, vt.at<float>(0) - vs.at<float>(2),
//                                             vt.at<float>(0) + vs.at<float>(2), vt.at<float>(1) * v.at<float>(1) + ca, vt.at<float>(1) - vs.at<float>(0),
//                                             vt.at<float>(2) - vs.at<float>(1), vt.at<float>(1) + vs.at<float>(0), vt.at<float>(2) * v.at<float>(2) + ca);

//     cv::Mat rvec;
//     cv::Rodrigues(R ,rvec);
//     return rvec;
// }