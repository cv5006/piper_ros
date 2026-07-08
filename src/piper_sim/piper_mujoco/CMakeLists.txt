cmake_minimum_required(VERSION 3.5)
project(piper_mujoco)

# CMake 컴파일 옵션 설정
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Python 스크립트 설치 규칙 생성
install(PROGRAMS
  scripts/piper_mujoco_ctrl.py
  scripts/piper_no_gripper_mujoco_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)

# 패키지 설명
ament_package()