#include "ros/ros.h"
#include "std_msgs/String.h"
#include "util.h"

#include <PI_Dragonfly.h>
#include <csignal>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dragonfly_data");
  ros::NodeHandle n("~/camera");

  PI::ImageReader imageReader;
  PI::ImageData imageleft_front{};

  ros::Publisher publisher = n.advertise<std_msgs::String>("left_front", 150);

  ros::Rate loop_rate(30);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    if (imageReader.Get_l_f(imageleft_front)) {
       ROS_INFO("%sRead image successfully!\n", imageleft_front.name.c_str());
       msg.data = imageleft_front.name;
       publisher.publish(msg);

    } else {
       ROS_ERROR("%sRead Failed retry\n", __FUNCTION__);
       ros::shutdown();
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
