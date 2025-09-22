#!/usr/bin/env python3
import os
import tempfile
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess,
    DeclareLaunchArgument, OpaqueFunction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Common args
    entity = LaunchConfiguration('entity')  # model name in Gazebo
    action = LaunchConfiguration('action')  # 'start' | 'respawn'
    use_external_gazebo = LaunchConfiguration('use_external_gazebo')  # 'true' => do NOT include Gazebo here

    def _make_urdf(_context):
        """xacro -> URDF xml string + temp file path."""
        pkg_share = get_package_share_directory('laser_tracker_tools')
        xacro_file = os.path.join(pkg_share, 'urdf', 'lasertracker.xacro')
        urdf_xml = xacro.process_file(xacro_file).toxml()
        tmp_urdf = os.path.join(tempfile.gettempdir(), 'lasertracker.urdf')
        with open(tmp_urdf, 'w') as f:
            f.write(urdf_xml)
        return urdf_xml, tmp_urdf

    def _maybe_gazebo_include(context):
        """Include Gazebo only if use_external_gazebo is 'false'."""
        if use_external_gazebo.perform(context) == 'true':
            return None
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'pause': 'false'}.items()
        )

    def _start_flow(context, *_, **__):
        """First boot: start Gazebo, spawn tracker, then load controllers."""
        urdf_xml, tmp_urdf = _make_urdf(context)
        actions = []

        # 1) Start Gazebo Classic via gazebo_ros launcher
        gazebo_inc = _maybe_gazebo_include(context)
        if gazebo_inc is not None:
            actions.append(gazebo_inc)

        # 2) robot_state_publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='laser_tracker',
            parameters=[
                {'robot_description': urdf_xml},
                {'frame_prefix': 'laser_tracker/'},
                {'use_sim_time': True},
            ],
            output='screen',
        )

        # 3) Spawn entity from /laser_tracker/robot_description
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace='laser_tracker',
            arguments=[
                '-file', tmp_urdf,
                '-entity', entity.perform(context),
            ],
            output='screen',
        )

        # 4) Wait for controller_manager then load: jsb -> joint1 -> joint2
        wait_cm = ExecuteProcess(
            cmd=['bash', '-lc',
                 'until ros2 service type /laser_tracker/controller_manager/list_controllers >/dev/null 2>&1; '
                 'do sleep 0.2; done; echo CM_READY'],
            output='screen',
        )
        sp_jsb = ExecuteProcess(cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/laser_tracker/controller_manager',
            '--controller-manager-timeout', '10.0'
        ], output='screen')
        sp_tilt = ExecuteProcess(cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'pan_tilt_controller',
            '--controller-manager', '/laser_tracker/controller_manager',
            '--controller-manager-timeout', '10.0'
        ], output='screen')

        # Chain: spawn -> wait_cm -> jsb -> tilt
        ev1 = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[wait_cm]))
        ev2 = RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=[sp_jsb]))
        ev3 = RegisterEventHandler(OnProcessExit(target_action=sp_jsb, on_exit=[sp_tilt]))

        actions += [rsp, spawn, ev1, ev2, ev3]
        return actions

    def _respawn_flow(context, *_, **__):
        """Hot-respawn: delete existing model, refresh RSP param, re-spawn, reload controllers."""
        urdf_xml, tmp_urdf = _make_urdf(context)

        pause = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/pause_physics', 'std_srvs/srv/Empty', '{}'],
            output='screen',
        )
        delete = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity',
                 "{name: '" + entity.perform(context) + "'}"],
            output='screen',
        )
        # Update robot_description on the running RSP
        set_rsp = ExecuteProcess(
            cmd=['bash', '-lc',
                 f'ros2 param set /laser_tracker/robot_state_publisher robot_description "$(<{tmp_urdf})"'],
            output='screen',
        )
        # Re-spawn from file (NOT topic)
        spawn = ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-file', tmp_urdf, '-entity', entity.perform(context)],
            output='screen',
        )
        wait_cm = ExecuteProcess(
            cmd=['bash', '-lc',
                 'until ros2 service type /laser_tracker/controller_manager/list_controllers >/dev/null 2>&1; '
                 'do sleep 0.2; done; echo CM_READY'],
            output='screen',
        )
        unpause = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
            output='screen',
        )

        sp_jsb = ExecuteProcess(cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/laser_tracker/controller_manager',
            '--controller-manager-timeout', '10.0'
        ], output='screen')
        sp_tilt = ExecuteProcess(cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'pan_tilt_controller',
            '--controller-manager', '/laser_tracker/controller_manager',
            '--controller-manager-timeout', '10.0'
        ], output='screen')

        ev1 = RegisterEventHandler(OnProcessExit(target_action=pause, on_exit=[delete]))
        ev2 = RegisterEventHandler(OnProcessExit(target_action=delete, on_exit=[set_rsp]))
        ev3 = RegisterEventHandler(OnProcessExit(target_action=set_rsp, on_exit=[spawn]))
        ev4 = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[wait_cm]))
        ev5 = RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=[unpause]))
        ev6 = RegisterEventHandler(OnProcessExit(target_action=unpause, on_exit=[sp_jsb]))
        ev7 = RegisterEventHandler(OnProcessExit(target_action=sp_jsb, on_exit=[sp_tilt]))

        return [pause, ev1, ev2, ev3, ev4, ev5, ev6, ev7]

    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='lasertracker'),
        DeclareLaunchArgument('action', default_value='start'),             # 'start' | 'respawn'
        DeclareLaunchArgument('use_external_gazebo', default_value='false'),# true => Gazebo는 외부에서 이미 실행됨
        OpaqueFunction(function=lambda ctx:
            _start_flow(ctx) if action.perform(ctx) == 'start' else _respawn_flow(ctx)
        ),
    ])