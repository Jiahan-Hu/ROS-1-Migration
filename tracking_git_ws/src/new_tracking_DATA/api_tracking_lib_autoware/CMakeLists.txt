cmake_minimum_required(VERSION 2.8.3)
project(api_tracking_lib_autoware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  pcl_ros
  geodesy
  sensor_msgs
  message_generation
  interactive_markers
  tf
  autoware_msgs
  geodesy
  tf2
  tf2_geometry_msgs
  common_lib
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2  common_lib
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(api_tracking_lib_autoware src/api_tracking_lib_autoware.cpp)
target_link_libraries(api_tracking_lib_autoware ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(api_tracking_lib_autoware_sim src/api_tracking_lib_autoware_sim.cpp)

target_link_libraries(api_tracking_lib_autoware_sim ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
	api_tracking_lib_autoware
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	PATTERN ".svn" EXCLUDE
)


