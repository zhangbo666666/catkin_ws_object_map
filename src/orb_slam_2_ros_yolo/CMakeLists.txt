cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2_ros_yolo)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()
#set(CMAKE_BUILD_TYPE DEBUG)
SET(CMAKE_BUILD_TYPE Release)

#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check for c++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  add_compile_options(-std=c++14)
else()
  message(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam2/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
roscpp
std_msgs
cv_bridge
image_transport
tf
sensor_msgs

octomap_ros
octomap_msgs

# add the msg
# mask_rcnn_ros
darknet_ros_msgs
yolact_ros_msgs
object_map_msgs
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.2.0 REQUIRED NO_MODULE)

find_package(Pangolin REQUIRED)
find_package(PCL 1.8.0 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/orb_slam2
${PROJECT_SOURCE_DIR}/orb_slam2/include
${PROJECT_SOURCE_DIR}/ros/include

${PROJECT_SOURCE_DIR}/ros/include/common
${PROJECT_SOURCE_DIR}/ros/include/object_builders

${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}

${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}

)


set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so

${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/orb_slam2/lib/libORB_SLAM2.so
${OpenCV_LIBS}
${catkin_LIBRARIES}

${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/ros/lib/librviz_visual_tools.so

${LIBS_ORBSLAM}
)

catkin_package (
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport tf sensor_msgs object_map_msgs octomap_ros octomap_msgs darknet_ros_msgs #mask_rcnn_ros
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)



add_executable (${PROJECT_NAME}_rgbd_dataset
ros/src/RGBDNode_dataset.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_rgbd_dataset ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_rgbd_dataset
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_rgbd_dataset_new
ros/src/RGBDNode_dataset_new.cc
ros/src/Node_dataset_new.cc
)
add_dependencies (${PROJECT_NAME}_rgbd_dataset_new ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_rgbd_dataset_new
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_publish_objects
ros/src/ObjectsPublish.cc
)
add_dependencies (${PROJECT_NAME}_publish_objects ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_publish_objects
${LIBS_ROS}
)


add_executable (${PROJECT_NAME}_objects_graph_extraction
        ros/src/ObjectsGraphExtraction.cc
        )
add_dependencies (${PROJECT_NAME}_objects_graph_extraction ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_objects_graph_extraction
        ${PROJECT_SOURCE_DIR}/ros/lib/libobject_builders_lib.so
        ${LIBS_ROS}
        )



# add_executable (${PROJECT_NAME}_semantic_navigation
# ros/src/ObjectsBasedSemanticNavigation.cc
# )
# add_dependencies (${PROJECT_NAME}_semantic_navigation ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}_semantic_navigation
# ${LIBS_ROS}
# )

# add_executable (${PROJECT_NAME}_load_pcd
# ros/src/LoadPCD.cc
# )
# add_dependencies (${PROJECT_NAME}_load_pcd ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}_load_pcd
# ${LIBS_ROS}
# )

# # show robot path
# add_executable (${PROJECT_NAME}_show_path
# ros/src/ShowPath.cpp
# )
# add_dependencies (${PROJECT_NAME}_show_path ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}_show_path
# ${LIBS_ROS}
# )

# ######################
# add_executable (${PROJECT_NAME}_test_gt
# ros/src/ObjectsPublishTestGT.cc
# )
# add_dependencies (${PROJECT_NAME}_test_gt ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}_test_gt
# ${LIBS_ROS}
# )
######################

add_executable (${PROJECT_NAME}_3d_object
ros/src/ObjectsNode.cc
ros/src/ObjectsBoundingBoxes.cc
ros/src/ObjectDatabase.cc
ros/src/PolygonIntersection.cpp
        )

#add_dependencies(${PROJECT_NAME}_3d_object
#  darknet_ros_msgs_generate_messages_cpp
#)
add_definitions(${PCL_DEFINITIONS})
add_dependencies(${PROJECT_NAME}_3d_object
  ${PROJECT_NAME}
  # mask_rcnn_ros_generate_messages_cpp
  # darknet_ros_msgs_generate_messages_cpp
  yolact_ros_msgs_generate_messages_cpp
  object_map_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_3d_object
${PROJECT_SOURCE_DIR}/ros/lib/libcalculateBb3dOverlap.so
${PROJECT_SOURCE_DIR}/ros/lib/libobject_builders_lib.so
${LIBS_ROS}
)

# Test 3DBB
# add_executable (${PROJECT_NAME}_test_3DBB
# ros/src/Test3DBB.cpp
# )
# add_dependencies (${PROJECT_NAME}_test_3DBB ${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}_test_3DBB
# ${LIBS_ROS}
#         ${PROJECT_SOURCE_DIR}/ros/lib/libobject_builders_lib.so
# )

