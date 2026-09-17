"""D1 — manipulability 타원체. (M1 · M3-3 · M3-4)

    ros2 launch piper_lecture_demo d1_manipulability.launch.py

슬라이더를 밀면 팔이 펴지고, 타원체가 납작해지고, sigma_min 이 0 으로 간다.

특이점 자세는 슬라이더로 정확히 맞추기 어렵다. 프리셋으로 재생한다:

    ros2 launch piper_lecture_demo d1_manipulability.launch.py preset:=wrist
    ros2 launch piper_lecture_demo d1_manipulability.launch.py preset:=elbow loop:=true

  preset=none(기본) 이면 슬라이더 GUI 가 뜨고, 그 외에는 프리셋 재생 노드가 뜬다.
  쓸 수 있는 값: none · good · home · wrist · elbow · shoulder

MoveIt 은 쓰지 않는다. 읽는 것은 URDF 하나뿐이라 MoveIt 을 아직 안 세운 팀도 그대로 돌린다.

**D2(작업영역 점구름)와 D6(IK 해 전수 탐색)도 여기서 뜬다.** 둘 다 URDF 만 읽으므로
MoveIt 스택이 필요 없다. D2 는 계산에 16 초쯤 걸려서 기본으로는 끄고, 필요할 때 켠다.

    ros2 launch piper_lecture_demo d1_manipulability.launch.py workspace:=true

점구름은 latch 되므로 계산이 끝나면 그대로 화면에 남는다.
자세 프리셋으로 팔을 옮겨가며 해를 다시 찾는 것이 이 스택에서 더 편하다 —
프리셋 버튼과 IK 버튼이 같은 패널에 있고, 타원체와 해가 한 화면에 같이 나온다.

    ros2 launch piper_lecture_demo d1_manipulability.launch.py ik:=false panel:=false

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
        DeclareLaunchArgument("ellipsoid", default_value="true",
                              description="타원체를 켠 채로 띄운다. "
                                          "강의 중에는 패널 버튼으로 바꾼다"),
        DeclareLaunchArgument("ik", default_value="true",
                              description="D6(IK 해 전수 탐색) 노드를 함께 띄운다"),
        DeclareLaunchArgument("workspace", default_value="false",
                              description="D2(작업영역 점구름)를 함께 띄운다. "
                                          "계산에 16 초쯤 걸린다"),
        DeclareLaunchArgument(
            "samples", default_value="[11, 13, 13, 5, 5, 1]",
            description="D2 의 관절 격자. 수직 단면만 보려면 [1, 61, 61, 9, 9, 1]"),
        DeclareLaunchArgument(
            "approach_tol_deg", default_value="25.0",
            description="D2 의 '원하는 자세' 허용 각. 손끝 z 축과 바닥 방향의 각도"),
        DeclareLaunchArgument(
            "voxel", default_value="0.02",
            description="D2 가 부피 비율을 낼 때 쓰는 복셀 한 변 [m]"),
        DeclareLaunchArgument("ik_valid", default_value="true",
                              description="초록(쓸 수 있는 해)을 켠 채로 띄운다"),
        DeclareLaunchArgument("panel", default_value="true",
                              description="버튼 패널을 함께 띄운다"),
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
                 "show": ParameterValue(LaunchConfiguration("ellipsoid"),
                                        value_type=bool),
             }]),

        # D2 — 작업영역 점구름. /joint_states 를 쓰지 않고 관절을 제 안에서
        # 훑어 latch 된 구름 둘을 낸다. 그래서 이 스택에 그냥 얹힌다.
        Node(package=PKG, executable="d2_workspace.py", output="screen",
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

        # D6 — 상주하며 부를 때마다 IK 해를 다시 찾는다. 결과는 링크 메시 Marker 라
        # rviz 기본 플러그인으로 그려진다 (MoveIt 도 SRDF 도 필요 없다).
        Node(package=PKG, executable="d6_ik_branches.py", output="screen",
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
             arguments=["-d", os.path.join(share, "config", "d1_manipulability.rviz")]),
    ])
