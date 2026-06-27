cmake_minimum_required(VERSION 3.0.2)
project(visualization_header_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  MarkerArrayHeader.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs visualization_msgs
)
