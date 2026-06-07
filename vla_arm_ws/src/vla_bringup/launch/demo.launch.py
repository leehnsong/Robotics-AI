from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': PathJoinSubstitution([
            FindPackageShare('arm_bringup'), 'worlds', 'scene.world'
        ])}.items()
    )

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([
            FindPackageShare('arm_bringup'), 'urdf', 'panda_gazebo.urdf.xacro'
        ])]),
        value_type=str
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'arm'],
        )
    ])

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

    move_group = TimerAction(period=7.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('arm_moveit_config'), '/launch/demo.launch.py'
            ])
        )
    ])

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

    orchestrator_node = TimerAction(period=11.0, actions=[
        Node(
            package='vla_orchestrator',
            executable='orchestrator_node',
            output='screen'
        )
    ])

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        joint_state_broadcaster,
        arm_controller,
        move_group,
        perception_node,
        planner_node,
        orchestrator_node,
    ])
