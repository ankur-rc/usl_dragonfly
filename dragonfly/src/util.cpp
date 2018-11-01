#include "ros/ros.h"

#include <iostream>
#include <exception>
#include <vector>
#include <string>

#include <PI_Dragonfly.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int width = 1280;
const int height = 720;

cv_bridge::CvImagePtr raw_to_bgr(PI::ImageData &raw_img, ros::Time tms, double &scale_x, double &scale_y, std::string header){

  cv::Mat yuv_img(height*3/2, width, CV_8UC1, raw_img.data.data());
  cv::Mat bgr_img;
  cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR_I420);
  cv::resize(bgr_img, bgr_img, cv::Size(), scale_x, scale_y, CV_INTER_AREA);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->header.stamp = tms;
  cv_ptr->header.frame_id = header;
  cv_ptr->encoding = "bgr8";
  cv_ptr->image = bgr_img;

  return cv_ptr;

}

void init_settings(ros::NodeHandle& n, int& fps, double& scale_x, 
                   double& scale_y, std::vector<int>& active_cams)
{
  
  ROS_INFO("Initializing parameters..");
  // init frames per second.
  if (n.param("fps", fps, 30)){
    ROS_INFO("Got 'fps': %d", fps);
  }else{
    ROS_ERROR("Failed to get param 'fps'");
  }

  // set image scaling: width
  if (n.param("scale_x", scale_x, 0.5)){
    ROS_INFO("Got 'scale_x': %f", scale_x);
  }else{
    ROS_ERROR("Failed to get param 'scale_x'");
  }

  // set image scaling: height
  if (n.param("scale_y", scale_y, 0.5)){
    ROS_INFO("Got 'scale_y': %f", scale_y);
  }else{
    ROS_ERROR("Failed to get param 'scale_y'");
  }

  // set active camera indices
  if (n.getParam("active_cams", active_cams)){
    ROS_INFO("Got 'active_cams'");
  }else{
    ROS_FATAL("Failed to get param 'active_cams'");
  }

  ROS_INFO("Parameter initialization completed.");
}

