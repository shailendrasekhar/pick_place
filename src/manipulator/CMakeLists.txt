cmake_minimum_required(VERSION 3.8)
project(manipulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(moveit2_demo manipulator/moveit2_demo.cpp)
target_include_directories(moveit2_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(moveit2_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core)

install(TARGETS moveit2_demo 
  DESTINATION lib/${PROJECT_NAME})
  
  install(DIRECTORY config launch manipulator meshes models rviz2 urdf worlds DESTINATION share/${PROJECT_NAME})
ament_package() 