import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def make_robot_description():
    """xacro 결과를 '한 줄'로 정리해서 반환.

    gazebo_ros2_control(0.4.x)은 robot_description(XML)을 controller_manager 에
    '--param robot_description:=<xml>' 형태로 넘기는데, rcl 이 그 값을 YAML 로 파싱한다.
    XML 선언(<?xml ...?>)·주석·개행이 있으면 파싱이 깨져 controller_manager 가 안 뜬다.
    → 선언/주석 제거 + 공백 1칸 축약으로 한 줄 URDF 를 만들어 robot_state_publisher 에 준다.
    (robot_state_publisher 가 들고 있는 이 값을 플러그인이 그대로 읽어 넘기므로 문제 해결)
    """
    xacro_path = os.path.join(
        get_package_share_directory('arm_bringup'),
        'urdf', 'panda_gazebo.urdf.xacro'
    )
    raw = subprocess.check_output(['xacro', xacro_path], text=True)
    s = re.sub(r'<\?xml.*?\?>', '', raw, flags=re.S)   # XML 선언 제거
    s = re.sub(r'<!--.*?-->', '', s, flags=re.S)        # 주석 제거
    s = re.sub(r'\s+', ' ', s).strip()                  # 개행/연속공백 → 공백 1칸
    return s


def generate_launch_description():

    # ── (1) Gazebo ──────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': PathJoinSubstitution([
            FindPackageShare('arm_bringup'), 'worlds', 'scene.world'
        ])}.items()
    )

    # ── (2) robot_state_publisher (정리된 한 줄 URDF) ─────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': make_robot_description()}]
    )

    # ── (3) 팔 스폰 (3초 후) ────────────────────────────────
    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'arm'],
        )
    ])

    # ── (4) 컨트롤러 (5초 후) ───────────────────────────────
    joint_state_broadcaster = TimerAction(period=5.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        )
    ])
    arm_controller = TimerAction(period=6.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
        )
    ])
    gripper_controller = TimerAction(period=6.5, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
        )
    ])

    # ── (5) MoveIt + RViz ───────────────────────────────────
    # arm_moveit_config 패키지가 이 워크스페이스에 없고, vla_control 은 MoveIt 없이
    # 프리셋 관절 궤적으로 동작하므로 MoveIt/RViz 블록은 생략한다.

    # ── (6) VLA 노드들 (10초 후) ────────────────────────────
    perception_node = TimerAction(period=10.0, actions=[
        Node(
            package='vla_perception',
            executable='perception_node',
            output='screen'
        )
    ])
    planner_node = TimerAction(period=10.0, actions=[
        Node(
            package='vla_planner',
            executable='llm_planner_node',
            output='screen'
        )
    ])
    control_node = TimerAction(period=10.0, actions=[
        Node(
            package='vla_control',
            executable='control_action_server',
            output='screen'
        )
    ])
    safety_node = TimerAction(period=10.0, actions=[
        Node(
            package='vla_orchestrator',
            executable='safety_node',
            output='screen'
        )
    ])
    task_manager_node = TimerAction(period=11.0, actions=[
        Node(
            package='vla_orchestrator',
            executable='task_manager_node',
            output='screen'
        )
    ])

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        perception_node,
        planner_node,
        control_node,
        safety_node,
        task_manager_node,
    ])
