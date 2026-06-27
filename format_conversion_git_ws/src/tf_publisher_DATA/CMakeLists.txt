cmake_minimum_required(VERSION 2.8.3)
project(tf_publisher)

# -mavx causes a lot of errors!!
#add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
#set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

## Compile as C++14
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  visualization_header_msgs
  geometry_msgs
  nav_msgs
  roscpp
  pcl_ros
  geodesy
  sensor_msgs
  message_generation
  interactive_markers
  tf
  tf2
  tf2_geometry_msgs
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

#generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs 
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

# add_executable(tf_publisher src/tf_publisher.cpp)
# target_link_libraries(tf_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tf_publisher_simple src/tf_publisher_sim_using_pc.cpp)
target_link_libraries(tf_publisher_simple ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(veh_tf_publisher src/veh_tf_publisher.cpp)
target_link_libraries(veh_tf_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
