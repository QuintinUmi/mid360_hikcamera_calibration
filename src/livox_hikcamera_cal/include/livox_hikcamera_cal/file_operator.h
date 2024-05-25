#ifndef _CSV_OPERATOR_H_
#define _CSV_OPERATOR_H_

#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>


namespace livox_hikcamera_cal
{
    class CsvOperator
    {
        public:

            CsvOperator(std::string file_path);
            ~CsvOperator();

            void setPath(std::string file_path);

            void writePointsToCSVOverwrite(const std::vector<geometry_msgs::Point32>& group1, const std::vector<geometry_msgs::Point32>& group2);
            void writePointsToCSVAppend(const std::vector<geometry_msgs::Point32>& group1, const std::vector<geometry_msgs::Point32>& group2);
            void deleteRowFromCSV(size_t rowIndex);
            void readPointsFromCSV(std::vector<geometry_msgs::Point32>& group1, std::vector<geometry_msgs::Point32>& group2);

        private:

            std::string file_path_;
    };

    class YamlOperator
    {
        public:

            YamlOperator(std::string file_path);
            ~YamlOperator();

            void setPath(std::string file_path);

            static void ensureFileExists(const std::string& filename);

            void writeExtrinsicsToYaml(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);
            bool readExtrinsicsFromYaml(Eigen::Matrix3f& R, Eigen::Vector3f& t);

        private:

            std::string file_path_;
    };
}


#endif