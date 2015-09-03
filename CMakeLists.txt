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
	object_recognition_msgs
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
	object_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES transparent_object_reconstruction
#  CATKIN_DEPENDS dynamic_reconfigure pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs object_recognition_msgs
#  DEPENDS system_lib
)

add_compile_options("-std=c++11")
#add_compile_options("-O0" "-g")

include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(tools src/tools.cpp)
target_link_libraries(tools ${catkin_LIBRARIES})

# Executables
add_executable(HoleVisualizer src/HoleVisualizer.cpp)
target_link_libraries(HoleVisualizer ${catkin_LIBRARIES} tools)
add_dependencies(HoleVisualizer transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pose2quatTrans src/pose2quatTrans.cpp)
target_link_libraries(pose2quatTrans ${catkin_LIBRARIES})

add_executable(HoleIntersector src/HoleIntersector.cpp)
target_link_libraries(HoleIntersector ${catkin_LIBRARIES} tools)
add_dependencies(HoleIntersector transparent_object_reconstruction_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(ExTraReconstructedObject src/ExTraReconstructedObject.cpp)
target_link_libraries(ExTraReconstructedObject ${catkin_LIBRARIES} tools)
add_dependencies(ExTraReconstructedObject transparent_object_reconstruction_gencfg)

# Generate ecto cells
pubsub_gen_wrap(${PROJECT_NAME} DESTINATION ${PROJECT_NAME} INSTALL)
add_dependencies(ecto_${PROJECT_NAME}_ectomodule ${PROJECT_NAME}_generate_messages_cpp)

ectomodule(hole_detection DESTINATION ${PROJECT_NAME} INSTALL
  src/ecto/HoleDetector.cpp
  src/ecto/module.cpp)
link_ecto(hole_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} tools)
add_dependencies(hole_detection_ectomodule ${PROJECT_NAME}_generate_messages_cpp)
