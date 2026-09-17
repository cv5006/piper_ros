cmake_minimum_required(VERSION 3.8)
project(piper_lecture_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  visualization_msgs
)

# D5 — 같은 두 자세, 두 경로 (관절 보간 vs 손끝 직선)
add_executable(d5_interp_compare src/d5_interp_compare.cpp)
ament_target_dependencies(d5_interp_compare ${DEPS})

install(TARGETS d5_interp_compare
  DESTINATION lib/${PROJECT_NAME})

# D1 · D2 — URDF 만 읽는 파이썬 도구 (의존성: numpy)
install(PROGRAMS
  scripts/d1_manipulability.py
  scripts/d1_preset.py
  scripts/d1_goto.py
  scripts/d2_workspace.py
  scripts/d3_obstacle.py
  scripts/d3_tcp_offset.py
  scripts/d6_ik_branches.py
  scripts/lecture_panel.py
  scripts/piper_real_adapter.py
  scripts/piper_driver_stub.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
