cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(object_detection_yolo)
rosbuild_init()


IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})
set(CMAKE_VERBOSE_MAKEFILE OFF)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0")
add_definitions(-w)
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.2.0 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system)
find_package( PCL 1.8.1 REQUIRED )
find_package (catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        image_transport
        tf
        sensor_msgs
        octomap_ros
        octomap_msgs
        darknet_ros_msgs
# add the msg
        object_map_msgs)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../../../
        ${PROJECT_SOURCE_DIR}/../../../include
        ${PROJECT_SOURCE_DIR}/../../../common
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        # /home/robot/catkin_ws_object_map/src/orb_slam_2_ros_yolo/ros/include/object_builders
        # /home/robot/catkin_ws_object_map/src/orb_slam_2_ros_yolo/ros/include/common
        # /home/robot/catkin_ws_object_map/src/orb_slam_2_ros_yolo/ros/include

)

catkin_package (
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport tf sensor_msgs darknet_ros_msgs object_map_msgs octomap_ros octomap_msgs
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}

        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
        #${PROJECT_SOURCE_DIR}/../../../lib/liborb_slam2.so
)

# Node for monocular camera
#rosbuild_add_executable(Mono
#src/ros_mono.cc
#)

#target_link_libraries(Mono
#${LIBS}
#)

# Node for monocular camera (Augmented Reality Demo)
#rosbuild_add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)

#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
#rosbuild_add_executable(Stereo
#src/ros_stereo.cc
#)

#target_link_libraries(Stereo
#${LIBS}
#)

# Node for RGB-D camera
#rosbuild_add_executable(RGBD
#src/ros_rgbd.cc
#)
#
#target_link_libraries(RGBD
#${LIBS}
#)


rosbuild_add_executable (${PROJECT_NAME}
src/RGBDNode.cc
src/Node.cc
)
#add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}
  object_map_msgs_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
${LIBS}
)


add_executable (object_detection src/object_detection.cc
        src/min_box_object_builder.cpp
        src/min_box_object_builder.hpp
        src/base_object_builder.hpp
        src/object_builder_manager.hpp
        )
#add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME})

target_link_libraries(object_detection
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
#        /home/robot/catkin_ws_object_map/src/orb_slam_2_ros_yolo/ros/lib/libobject_builders_lib.so
        )



