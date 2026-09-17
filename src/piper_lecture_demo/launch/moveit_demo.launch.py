"""MoveIt 데모 스택 — piper 의 런치를 그대로 부르고 rviz 설정만 우리 것을 넘긴다.

    ros2 launch piper_lecture_demo moveit_demo.launch.py

스택 자체는 만들지 않는다. piper 의 demo.launch.py 가 rsp · move_group · ros2_control ·
rviz 를 다 띄우므로 그것을 include 하고, rviz 설정만 바꿔 끼운다. 우리 설정은 piper 의
moveit.rviz 에 디스플레이 셋을 더한 것이다.

    Lecture Markers                 D3 · D5 가 내는 Marker
    IK solutions (within limits)    D6 — 초록, 기본 켜짐
    IK solutions (out of limits)    D6 — 빨강, 기본 꺼짐

D6 노드와 강의 패널도 같이 띄운다. 둘 다 인자로 끌 수 있다.

    ros2 launch piper_lecture_demo moveit_demo.launch.py panel:=false ik:=false

한 번 띄워두고 강의 내내 내리지 않는 것을 전제로 만들었다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "piper_lecture_demo"
MOVEIT_PKG = "piper_with_gripper_moveit"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    piper_demo = os.path.join(get_package_share_directory(MOVEIT_PKG),
                              "launch", "demo.launch.py")
    rviz_config = os.path.join(share, "config", "moveit_demo.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config,
                              description="기본값은 이 패키지의 moveit_demo.rviz"),
        DeclareLaunchArgument("ik", default_value="true",
                              description="D6(IK 해 탐색) 노드를 함께 띄운다"),
        DeclareLaunchArgument("panel", default_value="true",
                              description="버튼 패널을 함께 띄운다"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(piper_demo),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items()),

        # 상주하면서 버튼이나 서비스로 부를 때마다 다시 푼다
        Node(package=PKG, executable="d6_ik_branches.py", output="screen",
             condition=IfCondition(LaunchConfiguration("ik"))),

        Node(package=PKG, executable="lecture_panel.py", output="screen",
             condition=IfCondition(LaunchConfiguration("panel"))),
    ])
