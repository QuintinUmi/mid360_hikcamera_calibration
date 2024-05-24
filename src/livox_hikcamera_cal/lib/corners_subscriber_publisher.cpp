#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector>

#include "livox_hikcamera_cal/corners_subscriber_publisher.h"

namespace livox_hikcamera_cal
{
    CornersPublisherSubscriber::CornersPublisherSubscriber()
    {
        this->node_handle = ros::NodeHandle("CornersPublisherSubscriber");
        this->frame_id = std::string("livox_frame");
        this->subscribe_topic = std::string("/livox_hikcamera_cal/corners_sub");
        this->publish_topic = std::string("/livox_hikcamera_cal/corners_pub");

        init_subscribers();
        init_publishers(); 
    }
    CornersPublisherSubscriber::CornersPublisherSubscriber(ros::NodeHandle node_handle, std::string frame_id, std::string subscribe_topic, std::string publish_topic)
    {
        this->node_handle = node_handle;
        this->frame_id = frame_id;
        this->subscribe_topic = subscribe_topic;
        this->publish_topic = publish_topic;

        init_subscribers();
        init_publishers();    
    }

    CornersPublisherSubscriber::~CornersPublisherSubscriber()
    {
    }

    void CornersPublisherSubscriber::init_subscribers() 
    {
        
        this->corners_SUB = node_handle.subscribe<geometry_msgs::PolygonStamped>(this->subscribe_topic, 10, &CornersPublisherSubscriber::CornersSubCallBack, this);
        printf("Init Corners Subscriber [%s] Success!\n", this->subscribe_topic.c_str());
    }

    void CornersPublisherSubscriber::init_publishers()
    {
        this->corners_PUB = node_handle.advertise<geometry_msgs::PolygonStamped>(this->publish_topic, 10);
        printf("Init Corners Publisher [%s] Success!\n", this->subscribe_topic.c_str());
    }

    geometry_msgs::Point CornersPublisherSubscriber::convertPoint32ToPoint(const geometry_msgs::Point32& point32)
    {
        geometry_msgs::Point point;
        point.x = static_cast<double>(point32.x);
        point.y = static_cast<double>(point32.y);
        point.z = static_cast<double>(point32.z);
        return point;
    }
    geometry_msgs::Point32 CornersPublisherSubscriber::convertPointToPoint32(const geometry_msgs::Point& point)
    {
        geometry_msgs::Point32 point32;
        point32.x = static_cast<float>(point.x);
        point32.y = static_cast<float>(point.y);
        point32.z = static_cast<float>(point.z);
        return point32;
    }

    

    void CornersPublisherSubscriber::CornersSubCallBack(const geometry_msgs::PolygonStamped::ConstPtr& raw_corners)
    {
        this->received_corners_ = *raw_corners;
    }


    geometry_msgs::PolygonStamped CornersPublisherSubscriber::getCornersPolygonStamped()
    {
        return this->received_corners_;
    }
    std_msgs::Header CornersPublisherSubscriber::getCornersPolygonHeader()
    {
        return this->received_corners_.header;
    }
    geometry_msgs::Polygon CornersPublisherSubscriber::getCornersPolygon()
    {
        return this->received_corners_.polygon;
    }
    std::vector<geometry_msgs::Point32> CornersPublisherSubscriber::getCornersPoints32()
    {
        return this->received_corners_.polygon.points;
    }
    std::vector<geometry_msgs::Point> CornersPublisherSubscriber::getCornersPoints()
    {
        std::vector<geometry_msgs::Point> points;
        for (const auto& point32 : this->received_corners_.polygon.points) {
            points.push_back(convertPoint32ToPoint(point32));
        }

        return points;
    }


    std_msgs::Header CornersPublisherSubscriber::nowHeader()
    {
        std_msgs::Header header;
        header.frame_id = this->frame_id;
        header.stamp = ros::Time::now();

        return header;
    }


    void CornersPublisherSubscriber::publish(){}

    void CornersPublisherSubscriber::publish(geometry_msgs::PolygonStamped corners_pub)
    {
        this->corners_PUB.publish(corners_pub);
    } 
    void CornersPublisherSubscriber::publish(geometry_msgs::Polygon corners_polygon, std_msgs::Header header)
    {
        geometry_msgs::PolygonStamped corners_pub;
        corners_pub.polygon = corners_polygon;
        corners_pub.header = header;
        this->corners_PUB.publish(corners_pub);
    } 
    void CornersPublisherSubscriber::publish(std::vector<geometry_msgs::Point32> corners_point32, std_msgs::Header header)
    {
        geometry_msgs::PolygonStamped corners_pub;
        corners_pub.polygon.points = corners_point32;
        corners_pub.header = header;

        this->corners_PUB.publish(corners_pub);

    } 
    void CornersPublisherSubscriber::publish(std::vector<geometry_msgs::Point> corners_point, std_msgs::Header header)
    {
        geometry_msgs::PolygonStamped corners_pub;

        for(const auto& corner:corners_point)
        {
            corners_pub.polygon.points.emplace_back(this->convertPointToPoint32(corner));
        }
        corners_pub.header = header;

        this->corners_PUB.publish(corners_pub);

    } 

} 
