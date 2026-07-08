import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'piper'
    package_name = 'piper_description'
    urdf_name = "piper_no_gripper_description_gazebo.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')


    # urdf 파일에 $(find mybot) 구문이 있어 xacro로 한 번 컴파일해야 함
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())}

    # robot_state_publisher 노드를 실행하면 이 노드는 robot_description 토픽을 발행하며, 토픽 내용은 모델 파일 urdf의 내용이다
    # 또한 /joint_states 토픽을 구독하여 관절 데이터를 얻은 뒤 tf 및 tf_static 토픽을 발행한다.
    # 이 노드와 토픽들의 이름은 커스터마이징할 수 있는가?
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    # Launch the robot, robot_description 토픽을 통해 모델 내용을 가져와 gazebo에 모델을 생성한다
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # gazebo가 urdf를 로드할 때, urdf 설정에 따라 joint_states 노드를 실행하는가?
    # 관절 상태 발행기
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 경로 실행 컨트롤러, 즉 그 action인가?
    # 시스템은 my_group_controller 컨트롤러의 존재를 어떻게 아는가?
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )

    # 아래 두 개를 사용하는 것은 각 노드의 실행 순서를 제어하려는 의도로 보임
    # spawn_entity_cmd를 감시하다가 종료(완전히 실행)되면 load_joint_state_controller를 실행?
    close_evt1 =  RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # load_joint_state_controller를 감시하다가 종료(완전히 실행)되면 load_joint_trajectory_controller를 실행?
    # moveit은 gazebo가 여기서 제공하는 action과 어떻게 연결되는가??
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )

    ld = LaunchDescription()

    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
