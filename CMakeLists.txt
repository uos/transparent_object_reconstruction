cmake_minimum_required(VERSION 2.8.3)
project(transparent_object_reconstruction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)

generate_dynamic_reconfigure_options(
  cfg/CreateRays.cfg
  cfg/Intersec.cfg
  )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Holes.msg
)

add_service_files(
  FILES
  CombineClouds.srv
  CloudReset.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES transparent_object_reconstruction
#  CATKIN_DEPENDS dynamic_reconfigure pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(hole_region src/hole_region.cpp)
add_library(occupancy_grid src/occupancy_grid.cpp)
add_library(tools src/tools.cpp)

## Declare a cpp executable
add_executable(hole_test_pub src/hole_test_pub.cpp)
add_executable(hole_test_sub src/hole_test_sub.cpp)
add_executable(pcl_sub_test src/pcl_sub_test.cpp)
add_executable(create_rays src/create_rays.cpp)
add_executable(intersec src/intersec.cpp)
add_executable(label_cloud_test_pub src/label_cloud_test_pub.cpp)
add_executable(combine_clouds_server src/combine_clouds_server.cpp)
add_executable(combine_clouds_client src/combine_clouds_client.cpp)
add_executable(occp_test src/occp_test.cpp)
add_executable(hole_visualizer src/hole_visualizer.cpp)
add_executable(holes_msgs_test_pub src/holes_msgs_test_pub.cpp)
add_executable(pose2quatTrans src/pose2quatTrans.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(transparent_object_reconstruction_node transparent_object_reconstruction_generate_messages_cpp)
add_dependencies(create_rays transparent_object_reconstruction_gencfg)
add_dependencies(intersec transparent_object_reconstruction_gencfg)
add_dependencies(combine_clouds_server transparent_object_reconstruction_gencfg)
add_dependencies(combine_clouds_client transparent_object_reconstruction_gencfg)
add_dependencies(occupancy_grid hole_region)
add_dependencies(tools occupancy_grid hole_region)
add_dependencies(occp_test occupancy_grid hole_region)
add_dependencies(holes_msgs_test_pub transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(hole_visualizer transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hole_test_pub ${catkin_LIBRARIES})
target_link_libraries(hole_test_sub ${catkin_LIBRARIES})
target_link_libraries(pcl_sub_test ${catkin_LIBRARIES})
target_link_libraries(create_rays ${catkin_LIBRARIES})
target_link_libraries(intersec ${catkin_LIBRARIES})
target_link_libraries(label_cloud_test_pub ${catkin_LIBRARIES})
target_link_libraries(combine_clouds_server ${catkin_LIBRARIES})
target_link_libraries(combine_clouds_client ${catkin_LIBRARIES})
target_link_libraries(hole_region ${catkin_LIBRARIES})
target_link_libraries(occupancy_grid ${catkin_LIBRARIES} hole_region)
target_link_libraries(tools ${catkin_LIBRARIES} occupancy_grid hole_region)
target_link_libraries(occp_test ${catkin_LIBRARIES} occupancy_grid hole_region)
target_link_libraries(hole_visualizer ${catkin_LIBRARIES})
target_link_libraries(holes_msgs_test_pub ${catkin_LIBRARIES})
target_link_libraries(pose2quatTrans ${catkin_LIBRARIES})

ectomodule(transparent_object_reconstruction DESTINATION ${PROJECT_NAME} INSTALL
  src/HoleDetector.cpp)

link_ecto(transparent_object_reconstruction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
