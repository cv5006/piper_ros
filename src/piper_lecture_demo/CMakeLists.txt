cmake_minimum_required(VERSION 3.8)
project(piper_lecture_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# D1 — URDF 만 읽는 파이썬 도구. 빌드할 C++ 타깃이 없다.
install(PROGRAMS
  scripts/d1_manipulability.py
  scripts/d1_preset.py
  scripts/d1_goto.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
