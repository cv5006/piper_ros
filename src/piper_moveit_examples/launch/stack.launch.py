"""MoveIt 스택 실행 — 예제보다 먼저 띄워 두고 내리지 않는다.

    ros2 launch piper_moveit_examples stack.launch.py                       # mock (가짜 하드웨어)
    ros2 launch piper_moveit_examples stack.launch.py real:=true            # 실물 piper
    ros2 launch piper_moveit_examples stack.launch.py real:=true stub:=true # 실물 없이 어댑터 경로 시험

- 스택을 새로 만들지 않음 → piper_lecture_demo/moveit_demos.launch.py 재사용
- 강의용 부속(패널 · IK 해 탐색)은 끔
- real:=true
  - mock_components 대신 piper 드라이버 + 실물 어댑터(piper_real_adapter.py)
  - 사전에 CAN 활성화 필요: bash can_activate.sh can0 1000000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    demo = os.path.join(get_package_share_directory("piper_lecture_demo"),
                        "launch", "moveit_demos.launch.py")

    args = [
        DeclareLaunchArgument("real", default_value="false",
                              description="true: connect to the real piper"),
        DeclareLaunchArgument("stub", default_value="false",
                              description="with real:=true, use a driver stub instead of the driver"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("speed_percent", default_value="20",
                              description="Real-arm speed limit (1-100). Test well before raising it"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo),
        launch_arguments={
            "real": LaunchConfiguration("real"),
            "stub": LaunchConfiguration("stub"),
            "can_port": LaunchConfiguration("can_port"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "panel": "false",
            "ik": "false",
        }.items())

    return LaunchDescription(args + [stack])
