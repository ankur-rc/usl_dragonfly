#pragma once

#include "ros/ros.h"
#include <PI_Dragonfly.h>

#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

cv_bridge::CvImagePtr raw_to_bgr(PI::ImageData &raw_img, ros::Time tms,
                                 double &scale_x, double &scale_y, std::string header);

void init_settings(ros::NodeHandle& n, int& fps, double& scale_x, 
                   double& scale_y, std::vector<int>& active_cams);
