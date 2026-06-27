cmake_minimum_required(VERSION 2.8.3)
project(multi_modalities_sensor_fusion)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf_conversions
  nav_msgs
  cv_bridge
  image_transport
  visualization_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  jsk_rviz_plugins
  std_msgs
  vector_map_server
)

find_package(CUDA)
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED)

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
  INCLUDE_DIRS include
  DEPENDS EIGEN3 PCL 
  vector_map_server
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  jsk_rviz_plugins
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

###CAFFE
set(CAFFE_PATH "$ENV{HOME}/caffe/distribute")


  include_directories(${CUDA_INCLUDE_DIRS})

  IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
    link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
  endif ()

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CAFFE_PATH}/include
    ${PCL_INCLUDE_DIRS}
  )

  add_executable(multi_modalities_sensor_fusion
    src/multi_modalities_sensor_fusion_node.cpp
    src/cnn_segmentation.cpp
    src/cluster2d.cpp
    src/feature_generator.cpp
    src/ground_segmentation.cpp
    src/cluster.cpp
    src/lidar_cluster.cpp
  )
  target_link_libraries(multi_modalities_sensor_fusion
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CAFFE_PATH}/lib/libcaffe.so
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    glog
  )
  add_dependencies(multi_modalities_sensor_fusion
    ${catkin_EXPORTED_TARGETS}
  )

  install(TARGETS
    multi_modalities_sensor_fusion
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
  )

