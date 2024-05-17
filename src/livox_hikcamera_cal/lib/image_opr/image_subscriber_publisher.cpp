#include <ros/ros.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>

#include <mutex>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  

#include <image_transport/image_transport.h>

#include "livox_hikcamera_cal/image_opr/image_subscriber_publisher.h"

using namespace livox_hikcamera_cal::image_opr;

ImageSubscriberPublisher::ImageSubscriberPublisher() : node_handle(ros::NodeHandle()), img_it(node_handle) 
{
    // this->node_handle = ros::NodeHandle();
    // this->img_it = image_transport::ImageTransport(this->node_handle);
    this->subscribe_topic = std::string("/hikcamera/img_stream");
    this->publish_topic = std::string("/hikcamera/img_stream_proc");

    this->header_ = std_msgs::Header();

    init_subscribers();
    init_publishers(); 
}
ImageSubscriberPublisher::ImageSubscriberPublisher(ros::NodeHandle node_handle, std::string subscribe_topic, std::string publish_topic) : img_it(node_handle) 
{
    this->node_handle = node_handle;
    this->subscribe_topic = subscribe_topic;
    this->publish_topic = publish_topic;
    
    this->header_ = std_msgs::Header();

    init_subscribers();
    init_publishers();    
}

ImageSubscriberPublisher::~ImageSubscriberPublisher()
{
}

void ImageSubscriberPublisher::init_subscribers() 
{
    this->image_SUB = this->img_it.subscribe(this->subscribe_topic, 1,  &ImageSubscriberPublisher::ImageSubCallBack, this);
}

void ImageSubscriberPublisher::init_publishers()
{
    this->image_PUB = this->img_it.advertise(this->publish_topic, 1);
}



void ImageSubscriberPublisher::ImageSubCallBack(const sensor_msgs::ImageConstPtr& p_img_stream)
{
    // std::lock_guard<std::mutex> lock(this->mutex_);
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(p_img_stream, sensor_msgs::image_encodings::BGR8);
        this->header_ = cv_ptr->header;
        *this->image_ = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}


cv::Mat ImageSubscriberPublisher::getImage() const 
{
        // std::lock_guard<std::mutex> lock(this->mutex_);
        return this->image_->clone();
}




void ImageSubscriberPublisher::publish(const cv::Mat& image)
{
    if (!image.empty()) 
    {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(this->header_, "bgr8", image).toImageMsg();
        this->image_PUB.publish(msg);
        // ROS_INFO("Image published");
    }

} 




