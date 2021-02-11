cmake_minimum_required(VERSION 3.10)
project(tiago_webots_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(
  WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(
  $ENV{WEBOTS_HOME}/include/controller/c
  $ENV{WEBOTS_HOME}/include/controller/cpp
)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

#############
##  Build  ##
#############

add_library(${PROJECT_NAME} 
  src/robot_task.cc
)
target_include_directories(${PROJECT_NAME}  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} 
  ${WEBOTS_LIB}
)

ament_target_dependencies(${PROJECT_NAME} 
  action_msgs  
  nav2_bt_navigator
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

#add_executable(robot_task_node src/robot_task_node.cc)
#add_dependencies(robot_task_node ${PROJECT_NAME})
#target_link_libraries(robot_task_node ${PROJECT_NAME})


#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()