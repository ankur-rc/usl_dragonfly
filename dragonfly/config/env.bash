#!/bin/bash

export ROS_IP=10.42.0.13
export ROS_MASTER_URI=http://10.42.0.1:11311

source /home/ubuntu/catkin_ws/devel/setup.bash

exec "$@"
