"""URDF 만 읽는 데모 셋 — manipulability · workspace · ik_solutions.

    ros2 launch piper_lecture_demo urdf_demos.launch.py
    ros2 launch piper_lecture_demo urdf_demos.launch.py workspace:=true
    ros2 launch piper_lecture_demo urdf_demos.launch.py ik:=false panel:=false

같이 뜨는 것 — rsp · 슬라이더 GUI · 자세 드라이버 · manipulability · ik_solutions ·
패널 · rviz. workspace 는 계산 16 초라 기본으로 끈다.

자세 전환은 별도 노드다 (`goto_pose.py`). 특이점은 슬라이더로 맞추기 어렵다.
**시각화는 전부 감추기가 기본** — `ellipsoid:=true` · `ik_valid:=true` 로 켠다.

⚠ 실물을 연결하지 않은 상태로 띄울 것. /joint_states 가 명령 토픽이라 슬라이더가
  진짜 팔을 움직인다 (§6-①③).
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

    preset = LaunchConfiguration("preset")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="also launch the joint slider GUI"),
        DeclareLaunchArgument("preset", default_value="good",
                              description="startup pose"),
        DeclareLaunchArgument("duration", default_value="6.0",
                              description="time taken to move to the preset [s]"),
        DeclareLaunchArgument("loop", default_value="false",
                              description="play the preset back and forth"),
        DeclareLaunchArgument("scale", default_value="0.30",
                              description="ellipsoid size scale"),
        DeclareLaunchArgument("ellipsoid", default_value="false",
                              description="start with the ellipsoid already shown. "
                                          "default is hidden; reveal it with the panel button"),
        DeclareLaunchArgument("ik", default_value="true",
                              description="also launch ik_solutions (enumerate every IK solution)"),
        DeclareLaunchArgument("workspace", default_value="false",
                              description="also launch the workspace point cloud. "
                                          "takes about 16 s to compute"),
        DeclareLaunchArgument(
            "samples", default_value="[11, 13, 13, 5, 5, 1]",
            description="workspace joint sampling grid. use [1, 61, 61, 9, 9, 1] for a vertical slice"),
        DeclareLaunchArgument(
            "approach_tol_deg", default_value="25.0",
            description="workspace approach tolerance: angle between the TCP z axis and straight down"),
        DeclareLaunchArgument(
            "voxel", default_value="0.02",
            description="voxel edge length the workspace demo uses for the volume ratio [m]"),
        DeclareLaunchArgument("ik_valid", default_value="false",
                              description="start with the green (usable) solutions already "
                                          "shown. default is hidden"),
        DeclareLaunchArgument("panel", default_value="true",
                              description="also launch the button panel"),
        DeclareLaunchArgument(
            "readout", default_value="image",
            description="where to show the numbers: image (2D readout) / marker (3D text) / both / none"),
        DeclareLaunchArgument(
            "tcp_offset", default_value="[0.0, 0.0, 0.1358]",
            description="reference point: TCP position measured in the link6 frame [m]. "
                        "pass [0.0, 0.0, 0.0] to measure from the flange instead"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description}]),

        # 슬라이더 GUI — /joint_states 에 직접 쓰지 않고 비켜서 발행한다.
        # source_list 로 /joint_states 를 물려 두어 슬라이더가 프리셋 자세를 따라온다.
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen", condition=IfCondition(LaunchConfiguration("use_gui")),
             remappings=[("joint_states", "gui_joint_states")],
             parameters=[{"source_list": ["/joint_states"]}]),

        # 자세 드라이버 — /joint_states 의 유일한 발행자. 항상 뜬다.
        # 자세 전환은 goto_pose.py 로 한다.
        Node(package=PKG, executable="pose_presets.py", output="screen",
             parameters=[{
                 "preset": preset,
                 "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                 "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
             }]),

        Node(package=PKG, executable="manipulability.py", output="screen",
             parameters=[{
                 "urdf": URDF,
                 "scale": ParameterValue(LaunchConfiguration("scale"), value_type=float),
                 "tcp_offset": ParameterValue(LaunchConfiguration("tcp_offset"),
                                              value_type=List[float]),
                 "readout": LaunchConfiguration("readout"),
                 "show": ParameterValue(LaunchConfiguration("ellipsoid"),
                                        value_type=bool),
             }]),

        # workspace — 작업영역 점구름. /joint_states 를 쓰지 않고 관절을 제 안에서
        # 훑어 latch 된 구름 둘을 낸다. 그래서 이 스택에 그냥 얹힌다.
        Node(package=PKG, executable="workspace.py", output="screen",
             condition=IfCondition(LaunchConfiguration("workspace")),
             parameters=[{
                 "urdf": URDF,
                 "samples": ParameterValue(LaunchConfiguration("samples"),
                                           value_type=List[int]),
                 "approach_tol_deg": ParameterValue(
                     LaunchConfiguration("approach_tol_deg"), value_type=float),
                 "voxel": ParameterValue(LaunchConfiguration("voxel"), value_type=float),
                 "tcp_offset": ParameterValue(LaunchConfiguration("tcp_offset"),
                                              value_type=List[float]),
             }]),

        # ik_solutions — 상주하며 부를 때마다 IK 해를 다시 찾는다. 결과는 링크 메시 Marker 라
        # rviz 기본 플러그인으로 그려진다 (MoveIt 도 SRDF 도 필요 없다).
        Node(package=PKG, executable="ik_solutions.py", output="screen",
             parameters=[{
                 "urdf": URDF,
                 "show_valid": ParameterValue(LaunchConfiguration("ik_valid"),
                                              value_type=bool),
             }],
             condition=IfCondition(LaunchConfiguration("ik"))),

        Node(package=PKG, executable="lecture_panel.py", output="screen",
             condition=IfCondition(LaunchConfiguration("panel"))),

        Node(package="rviz2", executable="rviz2", output="log",
             condition=IfCondition(LaunchConfiguration("use_rviz")),
             arguments=["-d", os.path.join(share, "config", "urdf_demos.rviz")]),
    ])
