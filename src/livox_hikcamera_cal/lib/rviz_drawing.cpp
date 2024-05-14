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


    void RvizDrawing::addPoint(std::string object_id, float x, float y, float z, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker point;
        point.header.frame_id = this->frame_id_;
        point.header.stamp = ros::Time::now();
        point.ns = "points";
        point.id = this->updateMarkerId(object_id);
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
        this->updateObject(object_id, point);
    }
    void RvizDrawing::addPoint(std::string object_id, geometry_msgs::Point point_ip, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker point;
        point.header.frame_id = this->frame_id_;
        point.header.stamp = ros::Time::now();
        point.ns = "points";
        point.id = this->updateMarkerId(object_id);
        point.type = visualization_msgs::Marker::POINTS;
        point.action = visualization_msgs::Marker::ADD;
        point.points.emplace_back(point_ip);
        point.pose.orientation.w = 1.0;
        point.scale.x = scale;
        point.scale.y = scale;
        point.scale.z = scale;
        point.color.r = r;
        point.color.g = g;
        point.color.b = b;
        point.color.a = a;
        // std_msgs::ColorRGBA color;
        // color.r = r;
        // color.g = g;
        // color.b = b;
        // color.a = a;
        // point.colors.assign(point.points.size(), color);
        this->updateObject(object_id, point);
    }
    void RvizDrawing::addPoints(std::string object_id, std::vector<geometry_msgs::Point> points_ip, 
                                float scale, float r, float g, float b, float a) 
    {
        visualization_msgs::Marker points;
        points.header.frame_id = this->frame_id_;
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.id = this->updateMarkerId(object_id);
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
        // std_msgs::ColorRGBA color;
        // color.r = r;
        // color.g = g;
        // color.b = b;
        // color.a = a;
        // points.colors.assign(points.points.size(), color);
        this->updateObject(object_id, points);
    }


    void RvizDrawing::addLine(std::string object_id, float x1, float y1, float z1, float x2, float y2, float z2, 
                                float width, float r, float g, float b, float a)
    {
        visualization_msgs::Marker line;
        geometry_msgs::Point point;
        point.x = x1;
        point.y = y1;
        point.z = z1;
        line.points.emplace_back(point);
        point.x = x2;
        point.y = y2;
        point.z = z2;
        line.points.emplace_back(point);

        line.header.frame_id = frame_id_;
        line.header.stamp = ros::Time::now();
        line.ns = "lines";
        line.id = this->updateMarkerId(object_id);
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = width;
        line.color.r = r;
        line.color.g = g;
        line.color.b = b;
        line.color.a = a;
        this->updateObject(object_id, line);
    }
    void RvizDrawing::addLines(std::string object_id, const std::vector<geometry_msgs::Point>& points,
                                visualization_msgs::Marker::_type_type line_type, 
                                float width, float r, float g, float b, float a) {
        visualization_msgs::Marker lines;
        lines.header.frame_id = frame_id_;
        lines.header.stamp = ros::Time::now();
        lines.ns = "lines";
        lines.id = this->updateMarkerId(object_id);
        lines.type = line_type;
        lines.action = visualization_msgs::Marker::ADD;
        lines.scale.x = width;
        lines.points = points;
        lines.color.r = r;
        lines.color.g = g;
        lines.color.b = b;
        lines.color.a = a;
        this->updateObject(object_id, lines);
    }

    // void RvizDrawing::addImage(std::string object_id, const std::string& mesh_resource, float x, float y, float z,
    //                                 float scale_x, float scale_y, float scale_z) {
    //     visualization_msgs::Marker image;
    //     image.header.frame_id = frame_id_;
    //     image.header.stamp = ros::Time::now();
    //     image.ns = "images";
    //     image.id = this->updateMarkerId(object_id);
    //     image.type = visualization_msgs::Marker::MESH_RESOURCE;
    //     image.action = visualization_msgs::Marker::ADD;
    //     image.pose.position.x = x;
    //     image.pose.position.y = y;
    //     image.pose.position.z = z;
    //     image.scale.x = scale_x;
    //     image.scale.y = scale_y;
    //     image.scale.z = scale_z;
    //     image.mesh_resource = mesh_resource;
    //     image.color.a = 1.0; 
    //     this->updateObject(object_id, image);
    // }

    void RvizDrawing::deleteObject(std::string object_id)
    {
        auto it_object_id = this->objects_list_.find(object_id);
        if(it_object_id != this->objects_list_.end()) 
        {
            it_object_id->second.action = visualization_msgs::Marker::DELETE;
        }
    }
    void RvizDrawing::deleteAllObject()
    {
        visualization_msgs::Marker delete_all;
        delete_all.ns = "delete_all";
        delete_all.id = 0;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        this->objects_list_.clear();
        this->objects_list_["delete_all"] = delete_all;
    }

    void RvizDrawing::publish()
    {
        std::vector<std::map<std::string, visualization_msgs::Marker>::iterator> delete_objects;
        this->markers_.markers.clear();
        for (auto it_objects_list_ = this->objects_list_.begin(); it_objects_list_ != this->objects_list_.end(); ) 
        {
            if (it_objects_list_->second.action == visualization_msgs::Marker::DELETE || 
                it_objects_list_->second.action == visualization_msgs::Marker::DELETEALL ) 
            {
                this->markers_.markers.push_back(it_objects_list_->second);
                it_objects_list_ = this->objects_list_.erase(it_objects_list_);
            } 
            else 
            {
                this->markers_.markers.push_back(it_objects_list_->second);
                ++it_objects_list_;
                
            }
        }

        this->pub_.publish(this->markers_);

        // if(delete_objects.size() != 0)
        // {
        //     for(auto delete_object:delete_objects)
        //     {
        //         this->objects_list_.erase(delete_object); 
        //     }
        // }
    }


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


    int RvizDrawing::updateMarkerId(std::string object_id)
    {
        auto it_objects_list_ = this->objects_list_.find(object_id);
        if(it_objects_list_ != this->objects_list_.end())
        { 
            return it_objects_list_->second.id;
        }
        this->marker_id_ ++;
        return this->marker_id_;
    }
    // return 0 -> add, return 1 -> modify
    void RvizDrawing::updateObject(std::string object_id, const visualization_msgs::Marker& marker)
    {
        this->objects_list_[object_id] = marker;
    }
}