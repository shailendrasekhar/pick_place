cmake_minimum_required(VERSION 3.8)
project(manipulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(std_srvs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(moveit2_demo manipulator/moveit2_demo.cpp)
add_executable(planning_demo manipulator/planning_demo.cpp)
target_include_directories(moveit2_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(moveit2_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs 
  std_srvs
  linkattacher_msgs
  )

ament_target_dependencies(planning_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs 
  std_srvs
  )


install(TARGETS moveit2_demo planning_demo
  DESTINATION lib/${PROJECT_NAME})

install(
    PROGRAMS
    manipulator/sync_camera.py  # Ensure this path is correct
    DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY config launch manipulator meshes models rviz2 urdf worlds DESTINATION share/${PROJECT_NAME})
ament_package() 