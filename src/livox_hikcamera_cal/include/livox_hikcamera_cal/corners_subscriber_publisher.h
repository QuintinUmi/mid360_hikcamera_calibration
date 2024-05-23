#ifndef _CORNERS_SUBSCRIBER_PUBLISHER_H_
#define _CORNERS_SUBSCRIBER_PUBLISHER_H_

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <vector>


namespace livox_hikcamera_cal
{

    class CornersPublisherSubscriber
    {

        public:
            CornersPublisherSubscriber();
            CornersPublisherSubscriber(ros::NodeHandle node_handle, std::string frame_id, std::string subscribe_topic, std::string publish_topic);
            ~CornersPublisherSubscriber();

            geometry_msgs::PolygonStamped getCornersPolygonStamped();
            std_msgs::Header getCornersPolygonHeader();
            geometry_msgs::Polygon getCornersPolygon();
            std::vector<geometry_msgs::Point32> getCornersPoints32();
            std::vector<geometry_msgs::Point> getCornersPoints();

            std_msgs::Header nowHeader();

            geometry_msgs::Point convertPoint32ToPoint(const geometry_msgs::Point32& point32);
            geometry_msgs::Point32 convertPointToPoint32(const geometry_msgs::Point& point);
            
            void publish();
            void publish(geometry_msgs::PolygonStamped corners_pub);
            void publish(geometry_msgs::Polygon corners_pub, std_msgs::Header header);
            void publish(std::vector<geometry_msgs::Point32> corners_pub, std_msgs::Header header);
            void publish(std::vector<geometry_msgs::Point> corners_pub, std_msgs::Header header);

        protected:
            geometry_msgs::PolygonStamped received_corners_;

        private:
            ros::NodeHandle node_handle;
            std::string frame_id;
            ros::Subscriber corners_SUB;
            ros::Publisher corners_PUB;

            std::string subscribe_topic;
            std::string publish_topic;

            void init_subscribers();
            void init_publishers();

            void CornersSubCallBack(const geometry_msgs::PolygonStamped::ConstPtr& msg);

    };

}



#endif