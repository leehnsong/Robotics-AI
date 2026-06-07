# spawn_arm.launch.py
# 가이드 5-5의 "4개 터미널"을 하나로 묶은 런치 파일.
#   1) xacro 처리 -> robot_description (XML 주석 제거)
#   2) Gazebo + world(scene.world)
#   3) robot_state_publisher (robot_description 발행)
#   4) robot_description 토픽을 읽어 Panda 스폰
#   5) 스폰 끝나면 joint_state_broadcaster -> arm_controller 로드
#
# 주석 제거 이유: gazebo_ros2_control 이 robot_description 을 controller_manager 에
#   CLI 파라미터로 넘기는데, URDF 내 XML 주석이 있으면 파싱이 깨져 controller_manager 가
#   안 뜬다 (ros-controls/gazebo_ros2_control#295).
import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('arm_bringup')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg, 'urdf', 'panda_gazebo.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'scene.world')

    # Gazebo 는 URDF 의 package:// 메쉬를 model:// 로 바꿔 GAZEBO_MODEL_PATH 에서 찾는다.
    # panda 메쉬가 있는 share 디렉토리(=패키지 폴더의 부모)를 경로에 추가해야 팔이 그려진다.
    panda_share_parent = os.path.dirname(
        get_package_share_directory('moveit_resources_panda_description'))
    gazebo_model_path = panda_share_parent + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')

    # xacro 펼치기 + XML 주석 제거 -> 깨끗한 robot_description 문자열
    robot_xml = xacro.process_file(xacro_file).toxml()
    robot_xml = re.sub(r'<!--.*?-->', '', robot_xml, flags=re.DOTALL)

    # gzclient 는 package:// (또는 model://) 메쉬 경로를 못 풀어 팔이 안 보인다.
    # package:// 를 실제 절대경로(file://)로 치환하면 gzserver/gzclient 가 파일을 직접 연다.
    panda_share = get_package_share_directory('moveit_resources_panda_description')
    robot_xml = robot_xml.replace(
        'package://moveit_resources_panda_description', 'file://' + panda_share)

    robot_description = {'robot_description': robot_xml}

    gui = LaunchConfiguration('gui')

    # 1) Gazebo (factory/init 플러그인 포함 -> spawn_entity 가능)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true', 'gui': gui}.items(),
    )

    # 2) robot_state_publisher: robot_description 토픽 + TF 발행
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3) Panda 스폰 (robot_description 토픽에서 읽음)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'panda'],
        output='screen',
    )

    # 4) 컨트롤러 로드 (스폰 -> jsb -> arm_controller 순서 보장)
    load_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen',
    )
    load_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller'], output='screen',
    )
    
    # gripper_controller 자동 로드 
    load_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'], output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI(gzclient) 실행 여부'),
        # 온라인 모델 DB 조회 비활성화: model://sun, ground_plane 이 로컬 캐시에 있어도
        # Gazebo 가 models.gazebosim.org 접속을 시도하다 멈추는(행) 문제 방지.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # panda 메쉬(.dae) 를 gzclient 가 찾도록 모델 경로 추가 (안 하면 팔이 화면에 안 보임)
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm, on_exit=[load_gripper])),
    
    ])
