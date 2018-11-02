cmake_minimum_required(VERSION 3.0)
project(dragonfly)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED PATHS /usr/lib)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/lib/aarch64-linux-gnu/tegra/)

add_executable(${PROJECT_NAME}_data_streamer src/data_streamer_node.cpp src/util.cpp)

target_link_libraries(${PROJECT_NAME}_data_streamer 
                          ${catkin_LIBRARIES} 
                          ${CMAKE_CURRENT_SOURCE_DIR}/lib/libPI_DragonFly.a
                          ${OpenCV_LIBS} 
                          argus_socketclient 
                          EGL 
                          nvbuf_utils 
                          pthread
)
target_include_directories(${PROJECT_NAME}_data_streamer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(${PROJECT_NAME}_data_streamer PROPERTIES OUTPUT_NAME data_streamer PREFIX "")

#############
## Install ##
#############


#############
## Testing ##
#############


