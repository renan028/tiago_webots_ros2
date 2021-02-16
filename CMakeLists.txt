cmake_minimum_required(VERSION 3.10)
project(tiago_webots_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(
  WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)


# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  $ENV{WEBOTS_HOME}/include/controller/c
  $ENV{WEBOTS_HOME}/include/controller/cpp
)

set(${PROJECT_NAME}_dependencies
  action_msgs  
  nav2_bt_navigator
  nav2_map_server
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
)

#############
##  Build  ##
#############

add_library(${PROJECT_NAME} SHARED
  src/robot_task.cc
)

ament_target_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_dependencies}
)
target_link_libraries(${PROJECT_NAME} 
  ${WEBOTS_LIB}
)

add_executable(robot_task_node 
  src/robot_task_node.cc
)
ament_target_dependencies(robot_task_node 
  ${${PROJECT_NAME}_dependencies}
)
target_link_libraries(robot_task_node 
  ${PROJECT_NAME}  
)

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

install(TARGETS robot_task_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds resources config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME} 
)
ament_package()