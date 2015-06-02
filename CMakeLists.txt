cmake_minimum_required(VERSION 2.8.3)
project(transparent_object_reconstruction)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
  ecto
  ecto_ros
  ecto_pcl
  tf
  tf_conversions
)

generate_dynamic_reconfigure_options(
  cfg/CreateRays.cfg
  cfg/Intersec.cfg
)

add_message_files(
   FILES
   Holes.msg
)

add_service_files(
  FILES
  CombineClouds.srv
  CloudReset.srv
  HoleIntersectorReset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  pcl_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES transparent_object_reconstruction
#  CATKIN_DEPENDS dynamic_reconfigure pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(hole_region src/hole_region.cpp)
target_link_libraries(hole_region ${catkin_LIBRARIES})

add_library(occupancy_grid src/occupancy_grid.cpp)
target_link_libraries(occupancy_grid ${catkin_LIBRARIES} hole_region)

add_library(tools src/tools.cpp)
target_link_libraries(tools ${catkin_LIBRARIES} occupancy_grid hole_region)

# Executables
add_executable(create_rays src/create_rays.cpp)
target_link_libraries(create_rays ${catkin_LIBRARIES})
add_dependencies(create_rays transparent_object_reconstruction_gencfg)

add_executable(intersec src/intersec.cpp)
target_link_libraries(intersec ${catkin_LIBRARIES})
add_dependencies(intersec transparent_object_reconstruction_gencfg)

add_executable(combine_clouds_server src/combine_clouds_server.cpp)
target_link_libraries(combine_clouds_server ${catkin_LIBRARIES})
add_dependencies(combine_clouds_server transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(combine_clouds_client src/combine_clouds_client.cpp)
target_link_libraries(combine_clouds_client ${catkin_LIBRARIES})
add_dependencies(combine_clouds_client transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(hole_visualizer src/hole_visualizer.cpp)
target_link_libraries(hole_visualizer ${catkin_LIBRARIES})
add_dependencies(hole_visualizer transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pose2quatTrans src/pose2quatTrans.cpp)
target_link_libraries(pose2quatTrans ${catkin_LIBRARIES})

add_executable(HoleIntersector src/HoleIntersector.cpp)
target_link_libraries(HoleIntersector ${catkin_LIBRARIES} tools)
add_dependencies(HoleIntersector transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

# Tests
add_executable(hole_test_pub src/hole_test_pub.cpp)
target_link_libraries(hole_test_pub ${catkin_LIBRARIES})

add_executable(hole_test_sub src/hole_test_sub.cpp)
target_link_libraries(hole_test_sub ${catkin_LIBRARIES})

add_executable(label_cloud_test_pub src/label_cloud_test_pub.cpp)
target_link_libraries(label_cloud_test_pub ${catkin_LIBRARIES})

add_executable(pcl_sub_test src/pcl_sub_test.cpp)
target_link_libraries(pcl_sub_test ${catkin_LIBRARIES})

add_executable(holes_msgs_test_pub src/holes_msgs_test_pub.cpp)
target_link_libraries(holes_msgs_test_pub ${catkin_LIBRARIES})
add_dependencies(holes_msgs_test_pub transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(occp_test src/occp_test.cpp)
target_link_libraries(occp_test ${catkin_LIBRARIES} occupancy_grid hole_region)

# Generate ecto cells
pubsub_gen_wrap(transparent_object_reconstruction DESTINATION ${PROJECT_NAME} INSTALL)

ectomodule(hole_detection DESTINATION ${PROJECT_NAME} INSTALL
  src/ecto/HoleDetector.cpp
  src/ecto/module.cpp)
link_ecto(hole_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(hole_detection_ectomodule ${PROJECT_NAME}_generate_messages_cpp)
