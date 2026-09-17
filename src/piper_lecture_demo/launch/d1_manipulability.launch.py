"""D1 — manipulability 타원체. (M1 · M3-3 · M3-4)

    ros2 launch piper_lecture_demo d1_manipulability.launch.py

슬라이더를 밀면 팔이 펴지고, 타원체가 납작해지고, sigma_min 이 0 으로 간다.

특이점 자세는 슬라이더로 정확히 맞추기 어렵다. 프리셋으로 재생한다:

    ros2 launch piper_lecture_demo d1_manipulability.launch.py preset:=wrist
    ros2 launch piper_lecture_demo d1_manipulability.launch.py preset:=elbow loop:=true

  preset=none(기본) 이면 슬라이더 GUI 가 뜨고, 그 외에는 프리셋 재생 노드가 뜬다.
  쓸 수 있는 값: none · good · home · wrist · elbow · shoulder

MoveIt 은 쓰지 않는다. 읽는 것은 URDF 하나뿐이라 MoveIt 을 아직 안 세운 팀도 그대로 돌린다.

⚠ 실물을 연결하지 않은 상태로 띄울 것. 이 레포에서 /joint_states 는 명령 토픽이라
  연결돼 있으면 이 슬라이더가 진짜 팔을 움직인다 (강의 노트 §6-①③).
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "piper_lecture_demo"
URDF = os.path.join(get_package_share_directory("piper_description"),
                    "urdf", "piper_description.urdf")


def generate_launch_description():
    share = get_package_share_directory(PKG)

    with open(URDF, "r", encoding="utf-8") as f:
        robot_description = f.read()

    preset = LaunchConfiguration("preset")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="관절 슬라이더 GUI 를 함께 띄운다"),
        DeclareLaunchArgument("preset", default_value="good",
                              description="기동 자세"),
        DeclareLaunchArgument("duration", default_value="6.0",
                              description="프리셋까지 이동하는 시간 [s]"),
        DeclareLaunchArgument("loop", default_value="false",
                              description="프리셋을 왕복 재생한다"),
        DeclareLaunchArgument("scale", default_value="0.30",
                              description="타원체 크기 배율"),
        DeclareLaunchArgument(
            "readout", default_value="image",
            description="숫자 표시 위치. image(2D 숫자판) / marker(3D 텍스트) / both / none"),
        DeclareLaunchArgument(
            "tcp_offset", default_value="[0.0, 0.0, 0.1358]",
            description="기준점. link6 프레임에서 잰 손끝 위치 [m]. "
                        "[0.0, 0.0, 0.0] 을 주면 플랜지 기준이 된다"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description}]),

        # 슬라이더 GUI — /joint_states 에 직접 쓰지 않고 비켜서 발행한다.
        # source_list 로 /joint_states 를 물려 두어 슬라이더가 프리셋 자세를 따라온다.
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen", condition=IfCondition(LaunchConfiguration("use_gui")),
             remappings=[("joint_states", "gui_joint_states")],
             parameters=[{"source_list": ["/joint_states"]}]),

        # 자세 드라이버 — /joint_states 의 유일한 발행자. 항상 뜬다.
        # 자세 전환은 d1_goto.py 로 한다.
        Node(package=PKG, executable="d1_preset.py", output="screen",
             parameters=[{
                 "preset": preset,
                 "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                 "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
             }]),

        Node(package=PKG, executable="d1_manipulability.py", output="screen",
             parameters=[{
                 "urdf": URDF,
                 "scale": ParameterValue(LaunchConfiguration("scale"), value_type=float),
                 "tcp_offset": ParameterValue(LaunchConfiguration("tcp_offset"),
                                              value_type=List[float]),
                 "readout": LaunchConfiguration("readout"),
             }]),

        Node(package="rviz2", executable="rviz2", output="log",
             condition=IfCondition(LaunchConfiguration("use_rviz")),
             arguments=["-d", os.path.join(share, "config", "d1_manipulability.rviz")]),
    ])
