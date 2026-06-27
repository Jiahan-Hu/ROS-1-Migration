cmake_minimum_required(VERSION 3.0.2)
project(img_convert)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# find_package(cv_bridge REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES img_convert
  CATKIN_DEPENDS cv_bridge message_filters roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Specify libraries to link a library or executable target against
# ${PROJECT_NAME}_node
# target_link_libraries(image_converter_node
#   ${catkin_LIBRARIES}
#   cv_bridge
# )

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(compress2image src/compress2image.cpp)
target_link_libraries(compress2image ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
