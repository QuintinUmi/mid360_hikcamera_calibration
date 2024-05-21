#ifndef _IMAGE_PROCESS_H_
#define _IMAGE_PROCESS_H_

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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include "image_transport/image_transport.h"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include "livox_hikcamera_cal/image_opr/image_process.h"



namespace livox_hikcamera_cal
{

    namespace image_opr
    {

        class ImageProc
        {
            public:
                ImageProc();
                ~ImageProc();

                Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions);
        };

    }
}


#endif