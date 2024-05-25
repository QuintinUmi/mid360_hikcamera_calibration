#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/file_operator.h"

using namespace livox_hikcamera_cal;


CsvOperator::CsvOperator(std::string file_path)
{   
    this->setPath(file_path);
}

CsvOperator::~CsvOperator(){}


void CsvOperator::setPath(std::string file_path)
{
    this->file_path_ = file_path;
}

void CsvOperator::writePointsToCSVOverwrite(const std::vector<geometry_msgs::Point32>& group1, const std::vector<geometry_msgs::Point32>& group2) 
{
    std::ofstream outFile(this->file_path_, std::ofstream::trunc);  
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file for writing.\n";
        return;
    }
    
    outFile << "x1,y1,z1,x2,y2,z2\n";
    for (size_t i = 0; i < group1.size() && i < group2.size(); ++i) {
        outFile << group1[i].x << "," << group1[i].y << "," << group1[i].z << ","
                << group2[i].x << "," << group2[i].y << "," << group2[i].z << "\n";
    }
    
    outFile.close();
    std::cout << "Data successfully written to " << this->file_path_ << " (overwritten)\n";
}

void CsvOperator::writePointsToCSVAppend(const std::vector<geometry_msgs::Point32>& group1, const std::vector<geometry_msgs::Point32>& group2) 
{
    std::ofstream outFile(this->file_path_, std::ofstream::app);  
    if(!outFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file for writing.\n";
        return;
    }
    
    for(size_t i = 0; i < group1.size() && i < group2.size(); ++i) 
    {
        outFile << group1[i].x << "," << group1[i].y << "," << group1[i].z << ","
                << group2[i].x << "," << group2[i].y << "," << group2[i].z << "\n";
    }
    
    outFile.close();
    std::cout << "Data successfully appended to " << this->file_path_ << "\n";
}

void CsvOperator::deleteRowFromCSV(size_t rowIndex) 
{
    std::ifstream inFile(this->file_path_);
    std::vector<std::string> lines;
    std::string line;
    
    if(!inFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file for reading.\n";
        return;
    }
    
    while(getline(inFile, line)) 
    {
        lines.push_back(line);
    }
    
    inFile.close();

    if(rowIndex < 1 || rowIndex >= lines.size()) 
    {
        std::cerr << "Error: Row index out of range.\n";
        return;
    }
    
    std::ofstream outFile(this->file_path_);
    if(!outFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file for writing.\n";
        return;
    }
    
    for(size_t i = 0; i < lines.size(); ++i) 
    {
        if (i != rowIndex) {
            outFile << lines[i] << "\n";
        }
    }
    
    outFile.close();
    std::cout << "Row " << rowIndex << " deleted successfully from " << this->file_path_ << "\n";
}

void CsvOperator::readPointsFromCSV(std::vector<geometry_msgs::Point32>& group1, std::vector<geometry_msgs::Point32>& group2) 
{
    group1.clear();
    group2.clear();

    std::ifstream inFile(this->file_path_);
    if (!inFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file for reading.\n";
        return;
    }
    
    // Skip the header
    std::string line;
    std::getline(inFile, line);
    
    // Read the data
    while (std::getline(inFile, line)) 
    {
        std::stringstream ss(line);
        geometry_msgs::Point32 point1, point2;
        char delim;
        
        ss >> point1.x >> delim >> point1.y >> delim >> point1.z >> delim
           >> point2.x >> delim >> point2.y >> delim >> point2.z;
        
        group1.push_back(point1);
        group2.push_back(point2);
    }
    
    inFile.close();
    std::cout << "Data successfully read from " << this->file_path_ << std::endl;
}




YamlOperator::YamlOperator(std::string file_path)
{
    this->setPath(file_path);
}
YamlOperator::~YamlOperator(){}

void YamlOperator::setPath(std::string file_path)
{
    this->file_path_ = file_path;
}

void YamlOperator::ensureFileExists(const std::string& filename) 
{
    std::ifstream f(filename.c_str());
    if (!f.good()) {
        std::ofstream nf(filename.c_str());
        if (nf.good()) {
            nf << "root: {}" << std::endl;
        }
        nf.close();
        std::cout << "File created: " << filename << std::endl;
    } else {
        std::cout << "File exists: " << filename << std::endl;
    }
    f.close();
}

void YamlOperator::writeExtrinsicsToYaml(const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    YamlOperator::ensureFileExists(this->file_path_);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "Rotation" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < R.rows(); ++i) {
        for (int j = 0; j < R.cols(); ++j) {
            out << R(i, j);
        }
    }
    out << YAML::EndSeq;

    out << YAML::Key << "Translation" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < t.size(); ++i) {
        out << t(i);
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    // Write to file
    std::ofstream fout(this->file_path_);
    fout << out.c_str();
    fout.close();
}
bool YamlOperator::readExtrinsicsFromYaml(Eigen::Matrix3f& R, Eigen::Vector3f& t)
{
    try {
        YAML::Node config = YAML::LoadFile(this->file_path_);
        if (!config["Rotation"] || !config["Translation"]) {
            std::cerr << "Missing 'Rotation' or 'Translation' in YAML file." << std::endl;
            return false;
        }

        // Load rotation
        int idx = 0;
        for (int i = 0; i < R.rows(); ++i) {
            for (int j = 0; j < R.cols(); ++j) {
                R(i, j) = config["Rotation"][idx++].as<float>();
            }
        }

        // Load translation
        for (int i = 0; i < t.size(); ++i) {
            t(i) = config["Translation"][i].as<float>();
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return false;
    }
    return true;
}
