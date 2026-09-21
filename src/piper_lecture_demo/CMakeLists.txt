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
find_package(moveit_ros_planning REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  visualization_msgs
)

# path_compare — 같은 두 자세, 두 경로 (관절 보간 vs 손끝 직선)
add_executable(path_compare src/path_compare.cpp)
ament_target_dependencies(path_compare ${DEPS})

# jacobian_check — MoveIt 의 getJacobian() 과 강의 수식을 나란히 재서 대조한다.
# move_group 이 필요 없고 RobotModel 만 세우므로 moveit_ros_planning 을 쓴다.
add_executable(jacobian_check src/jacobian_check.cpp)
ament_target_dependencies(jacobian_check
  rclcpp moveit_core moveit_ros_planning ament_index_cpp)

install(TARGETS path_compare jacobian_check
  DESTINATION lib/${PROJECT_NAME})

# URDF 만 읽는 파이썬 도구 (의존성: numpy)
install(PROGRAMS
  scripts/manipulability.py
  scripts/pose_presets.py
  scripts/goto_pose.py
  scripts/workspace.py
  scripts/obstacle.py
  scripts/tcp_vs_flange.py
  scripts/ik_solutions.py
  scripts/lecture_panel.py
  scripts/piper_real_adapter.py
  scripts/piper_driver_stub.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md NOTES.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
