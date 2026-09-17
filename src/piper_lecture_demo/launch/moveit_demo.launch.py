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

실물 팔에 물리려면 real:=true 다. 그러면 mock_components 대신 진짜 드라이버와
어댑터가 뜬다 (piper_real_adapter.py 주석 참조).

    ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true
    ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true can_port:=can1
    ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true stub:=true   # 실물 없이

⚠ real:=true 는 **드라이버를 remap 없이** 띄운다. piper 의 런치는
  joint_ctrl_single 을 /joint_states 로 remap 하는데, 그 상태로는 어댑터가 내보낸
  상태가 그대로 명령으로 되돌아가 루프가 된다. 그래서 드라이버 노드를 직접 띄운다.

한 번 띄워두고 강의 내내 내리지 않는 것을 전제로 만들었다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "piper_lecture_demo"
MOVEIT_PKG = "piper_with_gripper_moveit"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    moveit_share = get_package_share_directory(MOVEIT_PKG)
    piper_demo = os.path.join(moveit_share, "launch", "demo.launch.py")
    rviz_config = os.path.join(share, "config", "moveit_demo.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config,
                              description="기본값은 이 패키지의 moveit_demo.rviz"),
        DeclareLaunchArgument("ik", default_value="true",
                              description="D6(IK 해 탐색) 노드를 함께 띄운다"),
        DeclareLaunchArgument("panel", default_value="true",
                              description="버튼 패널을 함께 띄운다"),
        DeclareLaunchArgument("real", default_value="false",
                              description="실물 팔에 물린다. mock_components 대신 "
                                          "드라이버와 어댑터가 뜬다"),
        DeclareLaunchArgument("stub", default_value="false",
                              description="real:=true 에서 진짜 드라이버 대신 대역을 "
                                          "쓴다. 실물 없이 시험할 때"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("speed_percent", default_value="20",
                              description="어댑터가 드라이버에 넘기는 속도 (1~100)"),

        # mock 스택 — rsp · move_group · ros2_control(mock) · rviz 를 한꺼번에
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(piper_demo),
            condition=UnlessCondition(LaunchConfiguration("real")),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items()),

        # 실물 스택 — 같은 것을 띄우되 ros2_control 자리를 어댑터가 대신한다.
        # demo.launch.py 를 통째로 쓸 수 없어서(그 안에 mock 이 있다) 조각으로 넣는다.
        *[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", f)),
            condition=IfCondition(LaunchConfiguration("real")))
          for f in ("static_virtual_joint_tfs.launch.py", "rsp.launch.py",
                    "move_group.launch.py")],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("real"), "' == 'true' and '",
                 LaunchConfiguration("use_rviz"), "' == 'true'"])),
            launch_arguments={"rviz_config": LaunchConfiguration("rviz_config")}.items()),

        # 진짜 드라이버. **remap 하지 않는다** — 위 경고 참조
        Node(package="piper", executable="piper_single_ctrl",
             name="piper_ctrl_single_node", output="screen",
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration("real"), "' == 'true' and '",
                  LaunchConfiguration("stub"), "' == 'false'"])),
             parameters=[{
                 "can_port": LaunchConfiguration("can_port"),
                 "auto_enable": True,
                 "gripper_exist": True,
                 "gripper_val_mutiple": 1,
             }]),

        # 실물 없이 시험할 때 쓰는 드라이버 대역
        Node(package=PKG, executable="piper_driver_stub.py", output="screen",
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration("real"), "' == 'true' and '",
                  LaunchConfiguration("stub"), "' == 'true'"]))),

        Node(package=PKG, executable="piper_real_adapter.py", output="screen",
             condition=IfCondition(LaunchConfiguration("real")),
             parameters=[{
                 "speed_percent": ParameterValue(
                     LaunchConfiguration("speed_percent"), value_type=int),
             }]),

        # 상주하면서 버튼이나 서비스로 부를 때마다 다시 푼다
        Node(package=PKG, executable="d6_ik_branches.py", output="screen",
             condition=IfCondition(LaunchConfiguration("ik"))),

        Node(package=PKG, executable="lecture_panel.py", output="screen",
             condition=IfCondition(LaunchConfiguration("panel"))),
    ])
