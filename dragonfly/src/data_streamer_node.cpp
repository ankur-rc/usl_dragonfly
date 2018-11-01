#include "ros/ros.h"
#include "util.h"

#include <iostream>
#include <exception>
#include <map>
#include <vector>
#include <string>

#include <PI_Dragonfly.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
 
int main(int argc, char **argv)
{

  int fps;
  std::vector<int> active_cams;
  double scale_x, scale_y;

  ros::init(argc, argv, "dragonfly");
  ros::NodeHandle n;

  init_settings(n, fps, scale_x, scale_y, active_cams);

  PI::ImageReader imageReader;
  PI::ImageData imageleft_front{};
  PI::ImageData imageright_front{};
  PI::ImageData imageleft_back{};
  PI::ImageData imageright_back{};

  image_transport::ImageTransport transport(n);
  image_transport::Publisher publisher_lf;
  image_transport::Publisher publisher_rf;
  image_transport::Publisher publisher_rb;
  image_transport::Publisher publisher_lb;
  
  for(auto i: active_cams){
    switch(i){
      case 0: publisher_lf = transport.advertise("/front_left", 5);
              break;
      case 1: publisher_rf = transport.advertise("/front_right", 5);
              break;
      case 2: publisher_rb = transport.advertise("/back_right", 5);
              break;
      case 3: publisher_lb = transport.advertise("/back_left", 5);
              break;
    }
  }
  
  ros::Rate loop_rate(fps);

  while (ros::ok())
  {
    if (imageReader.ReadAll(imageleft_front, imageleft_back, imageright_back, imageright_front)) {
      ROS_INFO("Read image successfully!\n");
      ros::Time tms = ros::Time::now();
      try{
        cv_bridge::CvImagePtr cv_ptr;
        for(auto i: active_cams){
          switch(i){
            case 0: cv_ptr = raw_to_bgr(imageleft_front, tms, scale_x, scale_y, "cam_lf");
                    publisher_lf.publish(cv_ptr->toImageMsg());
                    break;
            case 1: cv_ptr = raw_to_bgr(imageright_front, tms, scale_x, scale_y, "cam_rf");
                    publisher_rf.publish(cv_ptr->toImageMsg());
                    break;
            case 2: cv_ptr = raw_to_bgr(imageright_back, tms, scale_x, scale_y, "cam_rb");
                    publisher_rb.publish(cv_ptr->toImageMsg());
                    break;
            case 3: cv_ptr = raw_to_bgr(imageleft_back, tms, scale_x, scale_y, "cam_lb");
                    publisher_lb.publish(cv_ptr->toImageMsg());
                    break;
          }
        }
      } catch(std::exception &e) {
         ROS_ERROR("Error encountered: %s", e.what());
      }
    } else {
        ROS_ERROR("%sRead Failed!\n", __FUNCTION__);
        break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

