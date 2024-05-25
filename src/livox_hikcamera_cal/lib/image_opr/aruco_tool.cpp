#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"  
// #include "apriltag/apriltag.h"    
#include "opencv2/aruco/charuco.hpp"  

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
  
#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "livox_hikcamera_cal/conversion_bridge.h"

#include "livox_hikcamera_cal/image_opr/aruco_tool.h"


using namespace std;
using namespace livox_hikcamera_cal::image_opr;


ArucoM::ArucoM()
{
    this->dParameters = cv::aruco::DetectorParameters::create();
}
ArucoM::ArucoM(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength, cv::Mat cameraMatrix, cv::Mat disCoffes)
{   cv::aruco::DICT_6X6_1000;
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

void ArucoM::setDetectionParameters(int cornerRefinementMethod, int adaptiveThreshWinSizeMin, int adaptiveThreshWinSizeMax,
                                    int adaptiveThreshWinSizeStep, int adaptiveThreshConstant, 
                                    int minMarkerPerimeterRate, int maxMarkerPerimeterRate)
{
    this->dParameters->cornerRefinementMethod = cornerRefinementMethod;

    this->dParameters->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
    this->dParameters->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
    this->dParameters->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
    this->dParameters->adaptiveThreshConstant = adaptiveThreshConstant;

    this->dParameters->minMarkerPerimeterRate = minMarkerPerimeterRate;
    this->dParameters->maxMarkerPerimeterRate = maxMarkerPerimeterRate;
}

void ArucoM::create()
{
    this->aruco_map_init();
}

void ArucoM::aruco_map_init()
{
    for(int i = 0; i < this->selectedIds.size(); i++)
    {
        this->aruco_map[selectedIds[i]] = i;
    }
    
    printf("Aruco Map Init Success!\n");
}

// void ArucoM::release()
// {
//     if(this->aruco_hash != NULL){
//         int* tmp = this->aruco_hash;
//         this->aruco_hash = NULL;
//         delete tmp;
//     }
// }


float ArucoM::getMarkerRealLength(int markerId)
{
    if(this->aruco_map.find(markerId) == this->aruco_map.end())
    {
        return 1.0;
    }
    return this->markerRealLength[this->aruco_map.find(markerId)->second];
}
vector<int> ArucoM::getSelectedIds()
{
    return this->selectedIds;
}
int ArucoM::getSelectedIds(int index)
{
    return this->selectedIds.at(index);
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
    this->selectedIds.clear();
    this->selectedIds = selectedIds;
    this->aruco_map_init();
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
    
    rvecs.clear();
    tvecs.clear();
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
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, this->markerRealLength[this->aruco_map.find(targetId)->second], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
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

void ArucoM::ext_calib_single_arucos(cv::Mat &inputImage, int targetId, 
                                    vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs)
{
    vector<vector<cv::Point2f>> markerCorners, selectedCorners;
    vector<int> markerIds;
    int indexId;
    vector<cv::Vec3d> rvecs3d, tvecs3d;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
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
        cv::aruco::estimatePoseSingleMarkers(selectedCorners, this->markerRealLength[this->aruco_map.find(targetId)->second], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
        int vecSize = selectedCorners.size();
        for(int j = 0; j < vecSize; j++)
        {
            rvecs.emplace_back(rvecs3d[j]);
            tvecs.emplace_back(rvecs3d[j]);
        }
    }
    else
    {
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
    }
    
}


void ArucoM::ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int>& detectedIds)
{
    vector<vector<cv::Point2f>> markerCorners;
    vector<vector<vector<cv::Point2f>>> selectedCorners;
    vector<int> markerIds;
    int indexId;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    detectedIds.clear();
    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
    if(markerIds.empty())
    {
        printf("No marker detected!\n");
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        printf("Marker %d find!\n", markerIds[indexId]);
        if(this->aruco_map.find(markerIds[indexId]) != this->aruco_map.end())
        {
            printf("Marker %d is selected!\n", markerIds[indexId]);
            std::vector<std::vector<cv::Point2f>> temp_markerCorners;
            temp_markerCorners.emplace_back(markerCorners[indexId]);
            selectedCorners.emplace_back(temp_markerCorners);
            detectedIds.emplace_back(markerIds[indexId]);           
        }
        
    }
    if(!detectedIds.empty())
    {

        vector<cv::Vec3d> rvecs3d, tvecs3d;
        for(int detectedIds_index = 0; detectedIds_index < detectedIds.size(); detectedIds_index++)
        {
            cv::aruco::estimatePoseSingleMarkers(selectedCorners[detectedIds_index], this->markerRealLength[this->aruco_map.find(markerIds[detectedIds_index])->second], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
            
            int vecSize = selectedCorners[detectedIds_index].size();
            for(int j = 0; j < vecSize; j++)
            {
                rvecs.emplace_back((cv::Mat_<float>(3, 1) << rvecs3d[j][0], rvecs3d[j][1], rvecs3d[j][2]));
                tvecs.emplace_back((cv::Mat_<float>(3, 1) << tvecs3d[j][0], tvecs3d[j][1], tvecs3d[j][2]));
            }
        }
        
    }
    else
    {
        std::cout << "Detection Empty!\n" << std::endl;
        rvecs = vector<cv::Mat>{};
        tvecs = vector<cv::Mat>{};
    }
    
}

void ArucoM::ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Vec3d> &rvecs, vector<cv::Vec3d> &tvecs, vector<int>& detectedIds)
{
    vector<vector<cv::Point2f>> markerCorners;
    vector<vector<vector<cv::Point2f>>> selectedCorners;
    vector<int> markerIds;
    int indexId;
    cv::Mat vecData;
    
    rvecs.clear();
    tvecs.clear();
    detectedIds.clear();
    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
    if(markerIds.empty())
    {
        printf("No marker detected!\n");
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
        return;
    }
    for(indexId = 0; indexId < markerIds.size(); indexId++){
        // printf("Marker %d find!\n", markerIds[indexId]);
        if(this->aruco_map.find(markerIds[indexId]) != this->aruco_map.end())
        {
            // printf("Marker %d is selected!\n", markerIds[indexId]);
            std::vector<std::vector<cv::Point2f>> temp_markerCorners;
            temp_markerCorners.emplace_back(markerCorners[indexId]);
            selectedCorners.emplace_back(temp_markerCorners);
            detectedIds.emplace_back(markerIds[indexId]);           
        }
        
    }
    if(!detectedIds.empty())
    {

        vector<cv::Vec3d> rvecs3d, tvecs3d;
        for(int detectedIds_index = 0; detectedIds_index < detectedIds.size(); detectedIds_index++)
        {
            cv::aruco::estimatePoseSingleMarkers(selectedCorners[detectedIds_index], this->markerRealLength[this->aruco_map.find(markerIds[detectedIds_index])->second], this->cameraMatrix, this->disCoffes, rvecs3d, tvecs3d);
            
            int vecSize = selectedCorners[detectedIds_index].size();
            for(int j = 0; j < vecSize; j++)
            {
                rvecs.emplace_back(rvecs3d[j]);
                tvecs.emplace_back(tvecs3d[j]);
            }
        }
        
    }
    else
    {
        std::cout << "Detection Empty!\n" << std::endl;
        rvecs = vector<cv::Vec3d>{};
        tvecs = vector<cv::Vec3d>{};
    }
    
}

void ArucoM::estimate_average_pose(const vector<cv::Vec3d> &rvecs, const vector<cv::Vec3d> &tvecs, vector<int>& detectedIds, cv::Vec3d& averageRvec, cv::Vec3d& averageTvec)
{
    int count = 0;
    cv::Vec3d sumTvecs(0, 0, 0);
    for(const auto& tvec : tvecs) 
    {
        sumTvecs += tvec;
        count ++;
    }
    averageTvec = cv::Vec3d(sumTvecs[0] / count, sumTvecs[1] / count, sumTvecs[2] / count);


    std::vector<Eigen::Quaterniond> quaternions = ConversionBridge::rvecs3dToQuaternions(rvecs);

    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (const auto& q : quaternions) {
        Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
        M += q_vec * q_vec.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_decomposition(M);
    auto eigen_vectors = eigen_decomposition.eigenvectors();
    auto eigen_values = eigen_decomposition.eigenvalues();

    int max_index = 0;
    double max_eigen_values = eigen_values(0);
    for(int i = 1; i <= 4; i++)
    {
        if(eigen_values(i) > max_eigen_values) 
        {
            max_eigen_values = eigen_values(i);
            max_index = i;
        }
    }

    Eigen::Vector4d max_eigen_vector = eigen_vectors.col(max_index);

    averageRvec = ConversionBridge::quaternionToRvec3d(Eigen::Quaterniond(max_eigen_vector(0), max_eigen_vector(1), max_eigen_vector(2), max_eigen_vector(3)).normalized());

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