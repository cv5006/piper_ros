cmake_minimum_required(VERSION 3.8)
project(piper_moveit_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(DEPS
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_eigen
)

set(EXAMPLES
  ex01_robot_model
  ex02_collision_objects
  ex03_planning
  ex03_1_planner_compare
  ex04_execution
)

foreach(ex ${EXAMPLES})
  add_executable(${ex} src/${ex}.cpp)
  ament_target_dependencies(${ex} ${DEPS})
endforeach()

install(TARGETS ${EXAMPLES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
