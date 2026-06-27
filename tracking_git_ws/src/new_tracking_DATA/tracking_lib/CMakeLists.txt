cmake_minimum_required(VERSION 2.8.3)
project(tracking_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    feature_extractors_lib
    object_builders_lib
    std_msgs
    sensor_msgs
    geometry_msgs
    common_lib
    segmenters_lib
    roi_filters_lib
    feature_extractors_lib
)

## System dependencies are found with CMake's conventions
### PCL
find_package(PCL REQUIRED COMPONENTS
    common
    io
    filters
    features
    segmentation
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
### Eigen
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS object_builders_lib feature_extractors_lib
    #  DEPENDS system_lib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
    src/hm_tracking_worker.cpp
    src/distances/tracker_obsv_distance.cpp
    src/filters/robust_kalman_filter.cpp
    src/matchers/hungarian_tracker_obsv_matcher.cpp
    src/trackers/multi_object_tracker.cpp
    src/trackers/object_tracker.cpp
)

## http://mariobadr.com/creating-a-header-only-library-with-cmake.html
#add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE include/common)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

install(TARGETS
	${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	PATTERN ".svn" EXCLUDE
)
