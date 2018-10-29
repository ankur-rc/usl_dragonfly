#include "ros/ros.h"
#include "util.h"

#include <PI_Dragonfly.h>
#include <iostream>
#include <sensor_msgs/Image.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<exception>
 
const int WIDTH = 1280;
const int HEIGHT = 720;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dragonfly_data");
  ros::NodeHandle n("~/camera");

  PI::ImageReader imageReader;
  PI::ImageData imageleft_front{};

  ros::Publisher publisher = n.advertise<sensor_msgs::Image>("left_front", 10);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    sensor_msgs::Image img;

    if (imageReader.Get_l_f(imageleft_front)) {
       ROS_INFO("'Read image successfully!\n");
 
       //unsigned char raw_img_arr[1382400];
       //std::copy(imageleft_front.data.begin(), imageleft_front.data.end(), raw_img_arr);

	    try{
		 cv::Mat yuv_img(HEIGHT*3/2, WIDTH, CV_8UC1, imageleft_front.data.data());
		 cv::Mat bgr_img;
		 cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR_I420);

		 //img.image = imageleft_front.data;
		 //img.name = imageleft_front.name;
		 //img.timestamp = imageleft_front.tms;
		 //publisher.publish(img);
		 //cv::imwrite("/home/ubuntu/abc.jpg", bgr_img);

	    } catch(std::exception &e){
		cout<<e.what()<<endl;
		}

    } else {
       ROS_ERROR("%sRead Failed retry\n", __FUNCTION__);
       ros::shutdown();
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
