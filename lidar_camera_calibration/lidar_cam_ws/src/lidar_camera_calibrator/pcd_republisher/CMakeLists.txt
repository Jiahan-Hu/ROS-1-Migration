cmake_minimum_required(VERSION 2.8.3)
project(pcd_republisher)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pcd_republisher
  src/pcd_republisher.cpp
)

add_dependencies(pcd_republisher
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pcd_republisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS pcd_republisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
