#include "ros/ros.h"
#include "util.h"

#include <iostream>
#include <exception>

#include <PI_Dragonfly.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

 
const int width = 1280;
const int height = 720;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dragonfly_data");
  ros::NodeHandle n("~");

  PI::ImageReader imageReader;
  PI::ImageData imageleft_front{};

  image_transport::ImageTransport transport(n);
  image_transport::Publisher publisher = transport.advertise("/camera/left_front", 5);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    sensor_msgs::Image img;

    if (imageReader.Get_l_f(imageleft_front)) {
       ROS_INFO("Read image successfully!\n");

	    try{
		 cv::Mat yuv_img(height*3/2, width, CV_8UC1, imageleft_front.data.data());
		 cv::Mat bgr_img;
		 cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR_I420);

                 cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
                 cv_ptr->header.stamp = ros::Time::now();
                 cv_ptr->header.frame_id = "cam_lf";
                 cv_ptr->encoding = "bgr8";
                 cv_ptr->image = bgr_img;
                 
                 publisher.publish(cv_ptr->toImageMsg());

	    } catch(std::exception &e) {
		ROS_ERROR("Error encountered: %s", e.what());
	    }

    } else {
       ROS_ERROR("%sRead Failed!\n", __FUNCTION__);
       ros::shutdown();
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
