"""D2 — 작업영역 점구름 + 조건수 컬러맵. (M3-2)

    ros2 launch piper_lecture_demo d2_workspace.launch.py

기본 격자는 joint1 · joint2 · joint3 · joint5 를 쓴다. 계산에 10초 안팎이 걸리고,
구름은 latch 되므로 rviz 를 나중에 띄워도 받는다.

수직 단면만 보고 싶으면 (강의용으로 훨씬 읽기 쉽다):

    ros2 launch piper_lecture_demo d2_workspace.launch.py samples:="[1, 61, 61, 9, 9, 1]"

MoveIt 은 쓰지 않는다. URDF 와 numpy 뿐이다.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "piper_lecture_demo"
URDF = os.path.join(get_package_share_directory("piper_description"),
                    "urdf", "piper_description.urdf")


def generate_launch_description():
    share = get_package_share_directory(PKG)

    with open(URDF, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "samples", default_value="[11, 13, 13, 5, 5, 1]",
            description="관절별 샘플 수. 1 이면 그 관절을 0 으로 고정한다. "
                        "joint4 를 고정하면 접근 방향이 아래로 쏠려 결과가 부풀려진다"),
        DeclareLaunchArgument(
            "voxel", default_value="0.02",
            description="부피 비율을 낼 때 쓰는 복셀 한 변 [m]"),
        DeclareLaunchArgument(
            "tcp_offset", default_value="[0.0, 0.0, 0.1358]",
            description="기준점. link6 프레임에서 잰 손끝 위치 [m]"),
        DeclareLaunchArgument(
            "approach_tol_deg", default_value="25.0",
            description="'원하는 자세'의 허용 각. 손끝 z 축과 바닥 방향의 각도"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description}]),

        # 로봇을 영자세로 세워 두기 위한 것. 슬라이더는 D1 쪽에 있다.
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             output="screen"),

        Node(package=PKG, executable="d2_workspace.py", output="screen",
             parameters=[{
                 "urdf": URDF,
                 "samples": ParameterValue(LaunchConfiguration("samples"),
                                           value_type=List[int]),
                 "approach_tol_deg": ParameterValue(LaunchConfiguration("approach_tol_deg"),
                                                    value_type=float),
                 "voxel": ParameterValue(LaunchConfiguration("voxel"), value_type=float),
                 "tcp_offset": ParameterValue(LaunchConfiguration("tcp_offset"),
                                              value_type=List[float]),
             }]),

        Node(package="rviz2", executable="rviz2", output="log",
             condition=IfCondition(LaunchConfiguration("use_rviz")),
             arguments=["-d", os.path.join(share, "config", "d2_workspace.rviz")]),
    ])
