#ifndef _RVIZ_DRAWING_H_
#define _RVIZ_DRAWING_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace livox_hikcamera_cal
{
    class RvizDrawing
    {
        public:
            
            RvizDrawing();
            ~RvizDrawing();

            void initPublisher();

        private:



            ros::Publisher pub;
    };
}

#endif