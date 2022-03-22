cmake_minimum_required(VERSION 3.0.2)
project(s2r_pipeline)


find_package(Eigen3 REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
# )

include_directories("/usr/include/eigen3")
include_directories(
  include
  ${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED system filesystem date_time thread)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS

  actionlib
  geometric_shapes
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs

  tf2_ros
  roslib
  std_srvs
  tf

)

add_message_files(
   FILES
   pose.msg
 )

add_service_files(
	FILES
	TargetNumber.srv
)

# add_service_files(
#   FILES
#   sim2real_srv.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs rospy std_msgs turtlesim message_runtime)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/scripts/detect_grasp_place_server.py
  src/scripts/client.py
  src/scripts/toServer.py
  src/scripts/get_number_class_1.py
  src/scripts/get_number_class_pose_3.py
  src/scripts/get_number_pose_1.py
  src/scripts/get_roi_one.py
  src/scripts/get_roi_3.py
  src/scripts/grasp_cube_ddd.py
  src/scripts/place_cube_ddd.py
  src/scripts/predict_1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# catkin_install_python(PROGRAMS
#   scripts/detect_grasp_place_server.py
#   scripts/client.py
#   scripts/toServer.py
#   scripts/get_number_class_1.py
#   scripts/get_number_class_pose_3.py
#   scripts/get_number_pose_1.py
#   scripts/get_roi_1.py
#   scripts/get_roi_3.py
#   scripts/grasp_cube_ddd.py
#   scripts/place_cube_ddd.py
#   scripts/predict.py
add_executable(${PROJECT_NAME}_pipeline src/pipeline.cpp)

add_library(${PROJECT_NAME}_Navcore src/Navcore.cpp)

add_dependencies(${PROJECT_NAME}_pipeline ${PROJECT_NAME}_gencpp)

add_dependencies(${PROJECT_NAME}_Navcore ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}_pipeline
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_Navcore
)

target_link_libraries(${PROJECT_NAME}_Navcore
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
