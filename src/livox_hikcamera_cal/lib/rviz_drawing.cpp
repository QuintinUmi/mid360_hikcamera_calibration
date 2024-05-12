#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "livox_hikcamera_cal/rviz_drawing.h"

namespace livox_hikcamera_cal
{

    RvizDrawing::RvizDrawing()
    {
        this->topic_ = std::string("livox_hikcamera_cal/rviz_drawing");
        this->frame_id_ = std::string("livox_frame");
        this->initPublisher();
    }
    RvizDrawing::RvizDrawing(std::string topic, std::string frame_id)
    {
        this->topic_ = topic;
        this->frame_id_ = frame_id;
        this->initPublisher();
    }
    RvizDrawing::~RvizDrawing()
    {
        this->shutdownPublisher();
    }

    void RvizDrawing::setRvizDrawing(std::string topic, std::string frame_id)
    {
        this->shutdownPublisher();
        this->topic_ = topic;
        this->frame_id_ = frame_id;
        this->initPublisher();
    }


    void RvizDrawing::addPoint(float x, float y, float z, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker point;
        point.header.frame_id = this->frame_id_;
        point.header.stamp = ros::Time::now();
        point.ns = "points";
        point.id = this->object_id_ ++;
        point.type = visualization_msgs::Marker::SPHERE;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = z;
        point.scale.x = scale;
        point.scale.y = scale;
        point.scale.z = scale;
        point.color.r = r;
        point.color.g = g;
        point.color.b = b;
        point.color.a = a;
        this->addObject(point);
    }
    void RvizDrawing::addPoint(geometry_msgs::Point point_ip, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker point;
        point.header.frame_id = this->frame_id_;
        point.header.stamp = ros::Time::now();
        point.ns = "points";
        point.id = this->object_id_ ++;
        point.type = visualization_msgs::Marker::POINTS;
        point.action = visualization_msgs::Marker::ADD;
        point.points.emplace_back(point_ip);
        point.scale.x = scale;
        point.scale.y = scale;
        point.scale.z = scale;
        point.color.r = r;
        point.color.g = g;
        point.color.b = b;
        point.color.a = a;
        this->addObject(point);
    }
    void RvizDrawing::addPoints(std::vector<geometry_msgs::Point> points_ip, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker points;
        points.header.frame_id = this->frame_id_;
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.id = this->object_id_ ++;
        points.type = visualization_msgs::Marker::POINTS;
        points.action = visualization_msgs::Marker::ADD;
        points.points = points_ip;
        points.pose.orientation.w = 1.0;
        points.scale.x = scale;
        points.scale.y = scale;
        points.scale.z = scale;
        points.color.r = r;
        points.color.g = g;
        points.color.b = b;
        points.color.a = a;
        this->addObject(points);
    }

    // void RvizDrawing::addLine(const std::vector<geometry_msgs::Point>& points,
    //                             float width, float r, float g, float b, float a) {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = frame_id_;
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "lines";
    //     marker.id = marker_id_++;
    //     marker.type = visualization_msgs::Marker::LINE_STRIP;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.scale.x = width;
    //     marker.points = points;
    //     marker.color.r = r;
    //     marker.color.g = g;
    //     marker.color.b = b;
    //     marker.color.a = a;
    //     addMarker(marker);
    // }

    // void RvizDrawing::addImage(const std::string& mesh_resource, float x, float y, float z,
    //                                 float scale_x, float scale_y, float scale_z) {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = frame_id_;
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "images";
    //     marker.id = marker_id_++;
    //     marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = x;
    //     marker.pose.position.y = y;
    //     marker.pose.position.z = z;
    //     marker.scale.x = scale_x;
    //     marker.scale.y = scale_y;
    //     marker.scale.z = scale_z;
    //     marker.mesh_resource = mesh_resource;
    //     marker.color.a = 1.0; // Ensure this is visible
    //     addMarker(marker);
    // }


    void RvizDrawing::initPublisher()
    {
        this->shutdownPublisher();
        this->pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->topic_, 10);
    }
    void RvizDrawing::shutdownPublisher()
    {
        if(this->pub_)
        {
            this->pub_.shutdown();
            this->pub_ = ros::Publisher();
        }
    }

    void RvizDrawing::addObject(const visualization_msgs::Marker& marker)
    {
        this->objects_.markers.emplace_back(marker);
    }
}