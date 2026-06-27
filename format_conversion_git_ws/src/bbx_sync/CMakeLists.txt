cmake_minimum_required(VERSION 3.0.2)
project(bbx_sync)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
  visualization_header_msgs
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   visualization_msgs visualization_header_msgs
)


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
#  INCLUDE_DIRS include
  LIBRARIES bbx_sync
  # CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf visualization_msgs visualization_header_msgs
  CATKIN_DEPENDS roscpp rospy sensor_msgs tf visualization_msgs visualization_header_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(bbx_sync_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(two_bbx_sync_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(two_header_bbx_sync_repub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(bbx_sync_new src/bbx_sync_new.cpp)
target_link_libraries(bbx_sync_new ${catkin_LIBRARIES})
add_dependencies(bbx_sync_new ${catkin_EXPORTED_TARGETS})

add_executable(two_bbx_sync_new src/two_bbx_sync_new.cpp)
target_link_libraries(two_bbx_sync_new ${catkin_LIBRARIES})
add_dependencies(two_bbx_sync_new ${catkin_EXPORTED_TARGETS})

add_executable(two_header_bbx_sync_repub src/two_header_bbx_sync_repub.cpp)
target_link_libraries(two_header_bbx_sync_repub ${catkin_LIBRARIES})
add_dependencies(two_header_bbx_sync_repub ${catkin_EXPORTED_TARGETS})

add_executable(bbx2headerbbx src/bbx2headerbbx.cpp)
target_link_libraries(bbx2headerbbx ${catkin_LIBRARIES})
add_dependencies(bbx2headerbbx ${catkin_EXPORTED_TARGETS})

add_executable(bbx2headerbbxv2 src/bbx2headerbbx_v2.cpp)
target_link_libraries(bbx2headerbbxv2 ${catkin_LIBRARIES})
add_dependencies(bbx2headerbbxv2 ${catkin_EXPORTED_TARGETS})

# add_executable(bbx_sync src/bbx_sync_node.cpp)
# target_link_libraries(bbx_sync ${catkin_LIBRARIES})

# add_executable(two_bbx_sync src/two_bbx_sync.cpp)
# target_link_libraries(two_bbx_sync ${catkin_LIBRARIES})

