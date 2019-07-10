cmake_minimum_required(VERSION 2.8.3)
project(fusion_detector)

set (CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  image_transport
  autoware_msgs
  autoware_config_msgs
  geometry_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  jsk_recognition_msgs
  jsk_recognition_utils
  eigen_conversions
  visualization_msgs
  tf_conversions
  jsk_topic_tools
  image_geometry
)


find_package(PkgConfig REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        cv_bridge
        pcl_conversions
        image_transport
        autoware_msgs
        autoware_config_msgs
        geometry_msgs
        grid_map_ros
        grid_map_cv
        grid_map_msgs
        jsk_recognition_msgs
        jsk_recognition_utils
        eigen_conversions
        visualization_msgs
        tf_conversions
        jsk_topic_tools
        image_geometry
        INCLUDE_DIRS include
)

###########
## Build ##
###########
find_path(MIIVIIFUSION_INCLUDES miivii_util.h miivii_fusion.h   
          /opt/miivii/include)

include_directories( include
        lib
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${MIIVIIFUSION_INCLUDES} 
)
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories( ${catkin_LIB_DIRS} /opt/miivii/lib/)


## Declare a cpp executable
set(NODE_NAME ${PROJECT_NAME}_node)
add_executable(${PROJECT_NAME}_node
               src/fusion_detector_node.cpp
               src/fusion_detector.cpp
               src/draw_rects.cpp
               src/cluster.cpp
               src/gencolors.cpp
)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${catkin_LIB_DIRS} miivii_fusion
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS  ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
