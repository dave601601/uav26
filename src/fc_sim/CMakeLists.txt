cmake_minimum_required(VERSION 3.10)
project(fc_sim CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(fc_core REQUIRED)
find_package(fc_sim_msgs REQUIRED)

add_executable(fc_sim_node src/fc_sim_node.cpp)
ament_target_dependencies(fc_sim_node
  rclcpp
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  actuator_msgs
  fc_core
  fc_sim_msgs)

install(TARGETS fc_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/hover_pub.py
  scripts/flight_demo_pub.py
  scripts/teleop_pub.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
