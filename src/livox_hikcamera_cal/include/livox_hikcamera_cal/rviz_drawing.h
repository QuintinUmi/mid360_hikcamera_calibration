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

            

            void addPoint(std::string object_id, float x, float y, float z, 
                            float scale=1.0, float r=1.0, float g=1.0, float b=1.0, float a=1.0);
            void addPoint(std::string object_id, geometry_msgs::Point point_ip, 
                            float scale=1.0, float r=1.0, float g=1.0, float b=1.0, float a=1.0);
            void addPoints(std::string object_id, std::vector<geometry_msgs::Point> points_ip, 
                            float scale=1.0, float r=1.0, float g=1.0, float b=1.0, float a=1.0);

            void addLine(std::string object_id, float x1, float y1, float z1, float x2, float y2, float z2, 
                            float width=1.0, float r=1.0, float g=1.0, float b=1.0, float a=1.0);
            void addLines(std::string object_id, const std::vector<geometry_msgs::Point>& points,
                            visualization_msgs::Marker::_type_type line_type = visualization_msgs::Marker::LINE_STRIP, 
                            float width=1.0, float r=1.0, float g=1.0, float b=1.0, float a=1.0);
            
            // void addImage(std::string object_id, const std::string& mesh_resource, float x, float y, float z,
            //                         float scale_x, float scale_y, float scale_z);

            void deleteObject(std::string object_id);
            void deleteAllObject();


            void publish();


        private:
            
            void initPublisher();
            void shutdownPublisher();

            int updateMarkerId(std::string object_id);
            void updateObject(std::string object_id, const visualization_msgs::Marker& marker);

            std::string topic_;
            std::string frame_id_;  

            ros::NodeHandle nh_;
            ros::Publisher pub_;
            visualization_msgs::MarkerArray markers_;
            std::map<std::string, visualization_msgs::Marker> objects_list_;
            
            int marker_id_ = 0;
                
    };
}

#endif