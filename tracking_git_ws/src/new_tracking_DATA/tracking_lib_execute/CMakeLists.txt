cmake_minimum_required(VERSION 2.8.3)
project(tracking_lib_execute)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-std=c++14)

##
# Copyright (C) 2019 by AutoSense Organization. All rights reserved.
# Gary Chan <chenshj35@mail2.sysu.edu.cn>
##

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    std_msgs
    sensor_msgs
    geometry_msgs
    # ROS PCL
    #pcl_conversions
    #pcl_ros
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    segmenters_lib
    # needed for detection_node
    roi_filters_lib
    tracking_lib
    feature_extractors_lib
)

find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
### PCL
find_package(PCL REQUIRED COMPONENTS
    common
    io
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cmake_modules common_lib segmenters_lib roi_filters_lib tracking_lib feature_extractors_lib
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########


### Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

### Declare a C++ executable
add_executable(${PROJECT_NAME}_detection_node
    src/detection_node.cpp
)
add_executable(${PROJECT_NAME}_tracking_node
    src/tracking_node.cpp
)

add_executable(${PROJECT_NAME}_tracking_node_pc
    src/tracking_node_pc.cpp
)

add_executable(${PROJECT_NAME}_tracking_pc_node_new
    src/tracking_pc_node_new.cpp
)

add_executable(${PROJECT_NAME}_tracking_node_bbx
    src/tracking_node_bbx.cpp
)

add_executable(${PROJECT_NAME}_tracking_node_bbx_2
    src/tracking_node_bbx_2.cpp
)
add_executable(${PROJECT_NAME}_detection_node_pc
    src/detection_node_pc.cpp
)
# add_executable(${PROJECT_NAME}_KF_tracking_node
#    src/kf_bbx_tracking.cpp
#)


### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detection_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracking_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracking_node_pc
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracking_pc_node_new
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_tracking_node_bbx
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_tracking_node_bbx_2
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_detection_node_pc
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
#target_link_libraries(${PROJECT_NAME}_KF_tracking_node
#    ${catkin_LIBRARIES}
#    ${Eigen3_LIBRARIES}
#)

#############
## Install ##
#############
install(TARGETS
	${PROJECT_NAME}_detection_node
	${PROJECT_NAME}_tracking_node
	${PROJECT_NAME}_tracking_node_pc
	# ${PROJECT_NAME}_KF_tracking_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
