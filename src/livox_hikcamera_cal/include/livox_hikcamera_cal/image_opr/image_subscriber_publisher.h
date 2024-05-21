#ifndef _IMAGE_SUBSCRIBER_PUBLISHER_H_
#define _IMAGE_SUBSCRIBER_PUBLISHER_H_

#include <ros/ros.h>

#include <mutex>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <image_transport/image_transport.h>


namespace livox_hikcamera_cal
{

    namespace image_opr
    {
        class ImageSubscriberPublisher
        {

            public:
                ImageSubscriberPublisher();
                ImageSubscriberPublisher(ros::NodeHandle node_handle, std::string subscribe_topic=std::string("/hikcamera/img_stream"), std::string publish_topic=std::string("/hikcamera/img_stream_proc"));
                ~ImageSubscriberPublisher();

                cv::Mat getImage() const;
                
                void publish(const cv::Mat& image);


            private:
                ros::NodeHandle node_handle;
                image_transport::ImageTransport img_it;
                image_transport::Subscriber image_SUB;
                image_transport::Publisher image_PUB;

                std::string subscribe_topic;
                std::string publish_topic;

                std_msgs::Header header_;

                std::shared_ptr<cv::Mat> image_;
                // mutable std::mutex mutex_;

                void init_subscribers();
                void init_publishers();
                void ImageSubCallBack(const sensor_msgs::ImageConstPtr& p_img_stream);

        };

    }

}



#endif