cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

find_package(PCL REQUIRED)

#20231011 added, build release instead of debug
set(CMAKE_BUILD_TYPE "Release")

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(CUDA)
find_package(Eigen3 QUIET)
find_package(autoware_build_flags REQUIRED)
#0425 find_package(autoware_msgs REQUIRED)
#0425 find_package(autoware_config_msgs REQUIRED)

AW_CHECK_CUDA()

if (USE_CUDA)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
	novatel_oem7_msgs#0425
        pcl_conversions
        velodyne_pointcloud
        ndt_tku
        ndt_cpu
        #0425 autoware_health_checker
        jsk_rviz_plugins
        ${PCL_OPENMP_PACKAGES}
        #0425 autoware_msgs
        #0425 autoware_config_msgs
	geodesy
	ndt_omp #20231012
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        std_msgs
        velodyne_pointcloud
        autoware_msgs
        autoware_config_msgs
        ndt_tku
        ndt_cpu
        jsk_rviz_plugins
        ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
 ${autoware_config_msgs_INCLUDE_DIRS}
 ${autoware_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})

add_executable(ndt_matching_dataset nodes/ndt_matching/ndt_matching_dataset.cpp)
target_link_libraries(ndt_matching_dataset ${catkin_LIBRARIES})
add_dependencies(ndt_matching_dataset ${catkin_EXPORTED_TARGETS})

add_executable(ndt_matching_zzl nodes/ndt_matching/ndt_matching_zzl.cpp)
target_link_libraries(ndt_matching_zzl ${catkin_LIBRARIES})
add_dependencies(ndt_matching_zzl ${catkin_EXPORTED_TARGETS})

add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
add_dependencies(ndt_mapping ${catkin_EXPORTED_TARGETS})

## add for fusion node ##
add_executable(localization_fusion nodes/localization_fusion/localization_fusion.cpp)
target_link_libraries(localization_fusion ${catkin_LIBRARIES})
add_dependencies(localization_fusion ${catkin_EXPORTED_TARGETS})

## add for localization plot node ##
add_executable(localization_plot nodes/localization_plot/localization_plot.cpp)
target_link_libraries(localization_plot ${catkin_LIBRARIES})
add_dependencies(localization_plot ${catkin_EXPORTED_TARGETS})

if (USE_CUDA)
    target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(ndt_mapping PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(localization_fusion PRIVATE ${CUDA_INCLUDE_DIRS}) ## add for fusion node ##
	target_include_directories(localization_plot PRIVATE ${CUDA_INCLUDE_DIRS}) ## add for loca plot node ##
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_matching PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
    set_target_properties(ndt_mapping PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
	set_target_properties(localization_fusion PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP") ## add for fusion node ##
	set_target_properties(localization_plot PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP") ## add for loca plot node ##
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))


add_executable(approximate_ndt_mapping nodes/approximate_ndt_mapping/approximate_ndt_mapping.cpp)
target_link_libraries(approximate_ndt_mapping ${catkin_LIBRARIES})
add_dependencies(approximate_ndt_mapping ${catkin_EXPORTED_TARGETS})

add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(queue_counter ${catkin_EXPORTED_TARGETS})


add_executable(ndt_matching_tku nodes/ndt_matching_tku/ndt_matching_tku.cpp)
target_link_libraries(ndt_matching_tku ${catkin_LIBRARIES})
add_dependencies(ndt_matching_tku ${catkin_EXPORTED_TARGETS})

add_executable(ndt_mapping_tku nodes/ndt_mapping_tku/ndt_mapping_tku.cpp)
target_link_libraries(ndt_mapping_tku ${catkin_LIBRARIES})
add_dependencies(ndt_mapping_tku ${catkin_EXPORTED_TARGETS})

add_executable(mapping nodes/ndt_mapping_tku/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})
add_dependencies(mapping ${catkin_EXPORTED_TARGETS})


add_library(ndt_matching_monitor_lib SHARED
        nodes/ndt_matching_monitor/ndt_matching_monitor.h
        nodes/ndt_matching_monitor/ndt_matching_monitor.cpp
        )
target_include_directories(ndt_matching_monitor_lib PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
add_dependencies(ndt_matching_monitor_lib
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ndt_matching_monitor_lib ${catkin_LIBRARIES})

add_executable(ndt_matching_monitor nodes/ndt_matching_monitor/ndt_matching_monitor_node.cpp)
target_link_libraries(ndt_matching_monitor ndt_matching_monitor_lib ${catkin_LIBRARIES})

add_executable(icp_matching nodes/icp_matching/icp_matching.cpp)
target_link_libraries(icp_matching ${catkin_LIBRARIES})
add_dependencies(icp_matching ${catkin_EXPORTED_TARGETS})

install(TARGETS
            ndt_matching
	    ndt_matching_dataset
            ndt_matching_zzl
            ndt_mapping
            approximate_ndt_mapping
            ndt_mapping_tku
            mapping
            ndt_matching_monitor_lib
            ndt_matching_monitor
            icp_matching
            queue_counter
	    localization_fusion ## add for fusion node ##
	    localization_plot ## add for loca plot node ##
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
