"""예제 실행 — 스택(stack.launch.py)이 떠 있는 상태에서.

    ros2 launch piper_moveit_examples example.launch.py ex:=ex1
    ros2 launch piper_moveit_examples example.launch.py ex:=ex3 pause:=4.0

- ex:= 에 쓸 수 있는 값
  - ex1 · ex2 · ex3 · ex3_1 · ex4 (축약)
  - 1 · ex01 · ex01_robot_model 도 같음
- ros2 run 대신 launch 를 쓰는 이유
  - 예제 노드도 로봇 모델을 직접 들고 있어야 함 (FK · IK)
  - 필요한 파라미터: robot_description · robot_description_semantic · kinematics
  - MoveItConfigsBuilder 가 piper_with_gripper_moveit 설정에서 한꺼번에 만들어 줌
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

EXAMPLES = {
    "1": "ex01_robot_model",
    "2": "ex02_collision_objects",
    "3": "ex03_planning",
    "3_1": "ex03_1_planner_compare",
    "4": "ex04_execution",
}


def resolve(name):
    """ex1 · ex01 · 1 · ex3_1 · ex01_robot_model → 실행 파일 이름."""
    name = name.strip()
    if name in EXAMPLES.values():
        return name
    m = re.fullmatch(r"(?:ex)?0*(\d+)(?:_(\d+))?", name)
    if m:
        key = m.group(1) + (f"_{m.group(2)}" if m.group(2) else "")
        if key in EXAMPLES:
            return EXAMPLES[key]
    choices = ", ".join(f"ex{k} ({full})" for k, full in EXAMPLES.items())
    raise RuntimeError(f"ex:={name} — no such example. Choices: {choices}")


def launch_example(context):
    executable = resolve(LaunchConfiguration("ex").perform(context))
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit").to_moveit_configs()

    return [Node(
        package="piper_moveit_examples",
        executable=executable,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"pause": ParameterValue(LaunchConfiguration("pause"), value_type=float)},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ex", default_value="ex1",
                              description="Example: ex1 · ex2 · ex3 · ex3_1 · ex4 (or full name)"),
        DeclareLaunchArgument("pause", default_value="2.0",
                              description="Pause between steps [s] — time to look at rviz"),
        OpaqueFunction(function=launch_example),
    ])
