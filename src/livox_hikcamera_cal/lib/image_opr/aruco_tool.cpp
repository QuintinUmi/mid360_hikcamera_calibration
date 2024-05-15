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
#include "opencv2/aruco/charuco.hpp"  
  
#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "aruco_tool.h"
// #include "drawing_tool.h"
// #include "param_code.h"


using namespace std;
// using namespace drt;


ArucoM::ArucoM()
{
    this->dParameters = cv::aruco::DetectorParameters::create();
    this->aruco_hash = NULL;
}
ArucoM::ArucoM(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength, cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < markerRealLength.size(); i++)
    {
        this->markerRealLength.emplace_back(markerRealLength[i]);
    }
    this->set_camera_intrinsics(cameraMatrix, disCoffes);
}
ArucoM::ArucoM(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength, cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->markerDictionary = markerDictionary;
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < markerRealLength.size(); i++)
    {
        this->markerRealLength.emplace_back(markerRealLength[i]);
    }
    this->set_camera_intrinsics(cameraMatrix, disCoffes);
}
ArucoM::~ArucoM()
{
    // if(this->aruco_hash != NULL){
    //     int* tmp = this->aruco_hash;
    //     this->aruco_hash = NULL;
    //     delete tmp;
    // }
        
}

void ArucoM::create()
{
    this->aruco_hash_init();
}

void ArucoM::aruco_hash_init()
{
    if(this->aruco_hash != NULL){
        int* tmp = this->aruco_hash;
        this->aruco_hash = NULL;
        delete tmp;
    }
    int hashSize = 0;
    for(int i = 0; i < this->selectedIds.size(); i++)
    {
        hashSize = max(hashSize, selectedIds[i]);
    }
    this->aruco_hash = new int[hashSize + 1]();
    for(int i = 0; i < this->selectedIds.size(); i++)
    {
        this->aruco_hash[selectedIds[i]] = i;
    }
    printf("Aruco Hash Init Success!\n");
}

void ArucoM::release()
{
    if(this->aruco_hash != NULL){
        int* tmp = this->aruco_hash;
        this->aruco_hash = NULL;
        delete tmp;
    }
}



void ArucoM::set_aruco(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < markerRealLength.size(); i++)
    {
        this->markerRealLength.emplace_back(markerRealLength[i]);
    }
}
void ArucoM::set_aruco(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = markerDictionary;
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    for(int i = 0; i < markerRealLength.size(); i++)
    {
        this->markerRealLength.emplace_back(markerRealLength[i]);
    }
}
void ArucoM::sel_aruco_ids(vector<int> selectedIds)
{
    this->selectedIds = selectedIds;
    this->aruco_hash_init();
}
void ArucoM::set_aruco_real_length(vector<float> markerRealLength)
{
    for(int i = 0; i < markerRealLength.size(); i++)
    {
        this->markerRealLength.emplace_back(markerRealLength[i]);
    }
}
void ArucoM::set_camera_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;
}


vector<cv::Mat> ArucoM::generate_aruco_marker(int markerSize)
{
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
vector<cv::Mat> ArucoM::generate_aruco_marker(int dictionaryName, vector<int> selectedIds, int markerSize)
{
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(markerDictionary, selectedIds[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
void ArucoM::generate_aruco_inner(int markerSize)
{
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        this->markerImage.emplace_back(temp);
    }
}
void ArucoM::generate_aruco_inner(int dictionaryName, vector<int> selectedIds, int markerSize)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        this->markerImage.emplace_back(temp);
    }
}


void ArucoM::detect_aruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, vector<int> markerIds)
{
    // cv::aruco::DetectorParameters testParameters;
    // testParameters.minDistanceToBorder = 0;
    // testParameters.adaptiveThreshWinSizeMax = 1500;
    // cv::Mat arucoTest;
    // cv::aruco::drawMarker(this->markerDictionary, 4, 500, arucoTest);
    vector<vector<cv::Point2f>> rejectPoints;
    cv::aruco::detectMarkers(inputImage, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), markerCorners, markerIds);
    // if (markerIds.size() > 0)
    //     cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);
    
    // std::cout << rejectPoints.empty() << std::endl;
}

void ArucoM::ext_calib_single_arucos(cv::Mat &inputImage, int targetId, 
                                    vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs)
{
    vector<vector<cv::Point2f>> markerCorners, selectedCorners;
    vector<int> markerIds;
    int indexId;
    vector<cv::Vec3d> rvecs3d, tvecs3d;
    cv::Mat vecData;
    
    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
    if(markerIds.empty())
    {
        // printf("No marker detected!\n");
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        // printf("Marker %d check!\n", markerIds[indexId]);
        if(markerIds[indexId] == targetId)
            selectedCorners.emplace_back(markerCorners[indexId]);
    }
    if(!selectedCorners.empty())
    {
        // std::cout << this->cameraMatrix << std::endl;
        std::cout << markerIds[indexId] << std::endl;
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, this->markerRealLength[this->aruco_hash[targetId]], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
        int vecSize = selectedCorners.size();
        for(int j = 0; j < vecSize; j++)
        {
            rvecs.emplace_back((cv::Mat_<float>(3, 1) << rvecs3d[j][0], rvecs3d[j][1], rvecs3d[j][2]));
            tvecs.emplace_back((cv::Mat_<float>(3, 1) << tvecs3d[j][0], tvecs3d[j][1], tvecs3d[j][2]));
        }
    }
    else
    {
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
    }
    
}


void ArucoM::set_target_id_hash(vector<int> targetIds)
{
    if(this->targetId_hash != NULL){
        bool* tmp = this->targetId_hash;
        this->targetId_hash = NULL;
        delete tmp;
    }
    int hashSize = 0;
    for(int i = 0; i < targetIds.size(); i++)
    {
        hashSize = max(hashSize, targetIds[i]);
    }
    this->targetId_hash = new bool[hashSize + 1]();
    for(int i = 0; i < targetIds.size(); i++)
    {
        this->targetId_hash[targetIds[i]] = true;
    }
    this->targetIds = targetIds;
    printf("targetId_hash Init Success!\n");
}
void ArucoM::release_target_id_hash()
{
    if(this->targetId_hash != NULL){
        bool* tmp = this->targetId_hash;
        this->targetId_hash = NULL;
        delete tmp;
    }
}
void ArucoM::ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int> detectedIds)
{
    vector<vector<cv::Point2f>> markerCorners, selectedCorners;
    vector<int> markerIds;
    int indexId;
    vector<cv::Vec3d> rvecs3d, tvecs3d;
    cv::Mat vecData;
    
    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
    if(markerIds.empty())
    {
        printf("No marker detected!\n");
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        printf("Marker %d check!\n", markerIds[indexId]);
        if(this->targetId_hash[markerIds[indexId]])
        {
            selectedCorners.emplace_back(markerCorners[indexId]);
            detectedIds.emplace_back(markerIds[indexId]);
            break;             
        }
        
    }
    if(!selectedCorners.empty())
    {
        // std::cout << this->cameraMatrix << std::endl;
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, this->markerRealLength[indexId], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
        int vecSize = selectedCorners.size();
        for(int j = 0; j < vecSize; j++)
        {
            rvecs.emplace_back((cv::Mat_<float>(3, 1) << rvecs3d[j][0], rvecs3d[j][1], rvecs3d[j][2]));
            tvecs.emplace_back((cv::Mat_<float>(3, 1) << tvecs3d[j][0], tvecs3d[j][1], tvecs3d[j][2]));
        }
    }
    else
    {
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
    }
    
}


void ArucoM::aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, 
                                int dictionaryName, bool showImage)
{
    cv::Mat processImg(arucoMarkerImages[0].size(), CV_8U, cv::Scalar(255));
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    cv::String saveFileName;
    vector<vector<cv::Point2f>> markerCorners;
    vector<int> markerIds;
    // Draw3D d3d;
    vector<int> markerIdss;
    markerIdss.emplace_back(5);
    // arucoMarkerImages = this->generate_aruco_marker(dictionaryName, markerIdss, 500);
    for(int i = 0; i < arucoMarkerImages.size(); i++)
    { 
        processImg = cv::Mat(arucoMarkerImages[i].size(), CV_8U, cv::Scalar(255));
        
        // d3d.center_image_scale(arucoMarkerImages[i], processImg, 0.5, 0.5, 1, 0, cv::Scalar(255));


        float srcH = arucoMarkerImages[i].size().height;
        float srcW = arucoMarkerImages[i].size().width;
        float cX = srcW / 2;
        float cY = srcH / 2;

        float scaleX = 0.5;
        float scaleY = 0.5;
        int flags = 1;
        int borderMode = 0;
        cv::Scalar borderValue(255);

        cv::Point2f srcP[] = {  cv::Point2f(srcW, 0), 
                                cv::Point2f(srcW ,srcH), 
                                cv::Point2f(0 ,srcH)
                            };

        cv::Point2f dstP[] = {  cv::Point2f((srcW - cX) * scaleX + cX, (0 - cY) * scaleY + cY), 
                                cv::Point2f((srcW - cX) * scaleX + cX, (srcH - cY) * scaleY + cY), 
                                cv::Point2f((0 - cX) * scaleX + cX, (srcH - cY) * scaleY + cY)
                            };

        cv::Mat warpM = cv::getAffineTransform(srcP, dstP);
        cv::warpAffine(arucoMarkerImages[i], processImg, warpM, arucoMarkerImages[i].size(), flags, borderMode, borderValue);


        cv::aruco::detectMarkers(processImg, markerDictionary, markerCorners, markerIds);
        saveFileName = imageSavePath + cv::String("id_") + std::to_string(markerIds[0]) + 
                                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                                        cv::String(".") + imageFormat;
        // saveFileName = to_string(i) + cv::String(".png");
        cv::imwrite(saveFileName, arucoMarkerImages[i]);
        if(showImage){
            cv::imshow(cv::String("id_") + std::to_string(markerIds[0]) + 
                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                        cv::String(".") + imageFormat, 
                        arucoMarkerImages[i]);
            cv::waitKey(0);
        }
    }
}