#ifndef _RVIZ_DRAWING_H_
#define _RVIZ_DRAWING_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

namespace livox_hikcamera_cal
{
    
    class RvizDrawing
    {
        public:
            
            RvizDrawing();
            RvizDrawing(std::string topic, std::string frame_id);
            ~RvizDrawing();

            void setRvizDrawing(std::string topic, std::string frame_id);
            void initRvizDrawing();

            

            void addPoint(float x, float y, float z, 
                            float scale = 1.0, float r = 1.0, float g = 1.0, float b = 1.0, float a = 1.0);
            void addPoint(geometry_msgs::Point point_ip, 
                        float scale, float r, float g, float b, float a);
            void addPoints(std::vector<geometry_msgs::Point> points_ip, 
                            float scale = 1.0, float r = 1.0, float g = 1.0, float b = 1.0, float a = 1.0);

            // void addLine(const std::vector<geometry_msgs::Point>& points,
            //                     float width, float r, float g, float b, float a);

            // void addImage(const std::string& mesh_resource, float x, float y, float z,
            //                         float scale_x, float scale_y, float scale_z)

            void publish();


        private:
            
            void initPublisher();
            void shutdownPublisher();

            void addObject(const visualization_msgs::Marker& marker);

            std::string topic_;
            std::string frame_id_;  

            ros::NodeHandle nh_;
            ros::Publisher pub_;
            visualization_msgs::MarkerArray objects_;
            int object_id_ = 0;
                
    };
}

#endif