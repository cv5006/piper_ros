"""D5 — 같은 두 자세, 두 경로. (M4-1)

먼저 스택을 띄워두고:

    ros2 launch piper_with_gripper_moveit demo.launch.py \n        rviz_config:=$(ros2 pkg prefix piper_lecture_demo)/share/piper_lecture_demo/config/moveit_demo.rviz

그 다음 이것을 실행한다:

    ros2 launch piper_lecture_demo d5_interp_compare.launch.py

rviz 에서 Marker 자취 세 개를 비교한다 — 주황(관절 보간) · 파랑(손끝 직선) · 빨강(한계).

이 노드는 궤적의 경유점마다 FK 를 다시 계산해 손끝 자취를 만든다. 그래서 자기 쪽에도
로봇 모델이 필요하고, 아래에서 그것을 넘겨준다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PKG = "piper_lecture_demo"
MOVEIT_PKG = "piper_with_gripper_moveit"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name=MOVEIT_PKG)
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(package=PKG, executable="d5_interp_compare", output="screen",
             parameters=[
                 moveit_config.robot_description,
                 moveit_config.robot_description_semantic,
                 moveit_config.robot_description_kinematics,
             ]),
    ])
