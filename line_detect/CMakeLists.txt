cmake_minimum_required(VERSION 2.8.3)
project(line_detect)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
visualization_msgs
  message_generation

)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   points.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS} include ${line_detect_SOURCE_DIR}/include/line_detect
${OpenCV_INCLUDE_DIRS}
)



add_executable(line_detect_node_front ${line_detect_SOURCE_DIR}/src/main_lineDetect_front.cpp)
 add_dependencies(line_detect_node_front line_detect_front_generate_messages_cpp)

 target_link_libraries(line_detect_node_front
   ${catkin_LIBRARIES}
${OpenCV_LIBS}
 )

#add_executable(line_detect_node_behind ${line_detect_SOURCE_DIR}/src/main_lineDetect_behind.cpp)
# target_link_libraries(line_detect_node_behind
#  ${catkin_LIBRARIES}
#${OpenCV_LIBS}
# )
	add_executable(line_detect_node_left ${line_detect_SOURCE_DIR}/src/main_lineDetect_left.cpp)
add_dependencies(line_detect_node_left line_detect_left_generate_messages_cpp)
	 target_link_libraries(line_detect_node_left
	   ${catkin_LIBRARIES}
	${OpenCV_LIBS}
	 )

	add_executable(front_left_global_node ${line_detect_SOURCE_DIR}/src/front_left_global.cpp)
add_dependencies(front_left_global_node front_left_global_generate_messages_cpp)
	 target_link_libraries(front_left_global_node
	   ${catkin_LIBRARIES}
	${OpenCV_LIBS}
	 )
#add_executable(line_detect_node_right ${line_detect_SOURCE_DIR}/src/main_lineDetect_right.cpp)
#target_link_libraries(line_detect_node_right
# ${catkin_LIBRARIES}
#${OpenCV_LIBS}
# )
#add_executable(line_detect_node1 ${line_detect_SOURCE_DIR}/src/main1.cpp)
#add_executable(calcGlobalPosition_node ${line_detect_SOURCE_DIR}/src/main_calcGlobalPosition.cpp ${line_detect_SOURCE_DIR}/src/global.cpp)
#target_link_libraries(calcGlobalPosition_node
#  ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )
	#add_executable(pub_points_node ${line_detect_SOURCE_DIR}/src/test_pub.cpp)

	#target_link_libraries(pub_points_node
	#   ${catkin_LIBRARIES}
	# )
	 add_executable(df_node ${line_detect_SOURCE_DIR}/src/df.cpp)
add_dependencies(df_node df_generate_messages_cpp)
	 target_link_libraries(df_node
	   ${catkin_LIBRARIES}
	   ${OpenCV_LIBS}
	 )

 add_executable(points_and_lines ${line_detect_SOURCE_DIR}/src/test_rviz.cpp)
add_dependencies(points_and_lines points_and_lines_generate_messages_cpp)
 target_link_libraries(points_and_lines
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )


set(CMAKE_BUILD_TYPE Release)
