#!/usr/bin/env python

"""Launch Webots IsfrFullRobot driver with optional Nav2."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

def generate_launch_description():

    # --- Launch arguments ---
    launch_webots = LaunchConfiguration('launch_webots', default='true')
    launch_nav2 = LaunchConfiguration('launch_nav2', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world', default='webots_living_room_world.wbt')

    # --- Package paths ---
    pkg_bringup = get_package_share_directory('isfr_bot_bringup')
    pkg_desc = get_package_share_directory('isfr_bot_description')

    # --- Files ---
    world_file = PathJoinSubstitution([pkg_bringup, 'worlds', world_file_name])
    proto_path = os.path.join(pkg_desc, 'protos')
    windows_proto_path = proto_path.replace('/home/', '//wsl.localhost/Ubuntu/home/')

    # --- Webots Launcher ---
    node_webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True,
        env={'WEBOTS_ROBOT_PATH': windows_proto_path},
    )

    # --- ROS2 Control spawners ---
    controller_manager_prefix = '' if os.name != 'nt' else 'python.exe'
    controller_manager_timeout = ['--controller-manager-timeout', '50']

    diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout
    )

    ros_control_spawners = [diffdrive_spawner, joint_state_spawner]

    # --- WebotsController ---
    # The controller will automatically publish /robot_description and joint_states
    turtlebot_driver = WebotsController(
        robot_name='IsfrFullRobot',
        parameters=[{
            'use_sim_time': use_sim_time,
            'set_robot_state_publisher': True
        }],
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped'),
            ('/diffdrive_controller/odom', '/odom')
        ],
        respawn=True
    )

    # --- Twist stamper ---
    # This node republishes /cmd_vel -> /cmd_vel_stamped for controllers
    twist_stamper = Node(
        package='isfr_bot_bringup',  # your package for twist handling
        executable='twist_stamper',
        output='screen'
    )

    # --- Optional Nav2 ---
    navigation_nodes = []
    if 'turtlebot3_navigation2' in get_packages_with_prefixes():
        nav_map_file = os.path.join(pkg_bringup, 'resource', 'map.yaml')
        nav_params_file = os.path.join(pkg_bringup, 'resource', 'nav2_params.yaml')
        turtlebot_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch', 'navigation2.launch.py'
            )),
            launch_arguments=[
                ('map', nav_map_file),
                ('params_file', nav_params_file),
                ('use_sim_time', use_sim_time),
                ('autostart', 'true')
            ],
            condition=launch.conditions.IfCondition(launch_nav2)
        )
        navigation_nodes.append(turtlebot_nav)

    # --- Wait for WebotsController before starting controllers / Nav2 ---
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners + navigation_nodes
    )

    # --- Shutdown handler ---
    shutdown_on_exit = RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=node_webots,
            on_exit=[EmitEvent(event=launch.events.Shutdown())]
        )
    )

    # --- Launch Description ---
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('launch_webots', default_value='true', description='Launch Webots simulation'),
        DeclareLaunchArgument('launch_nav2', default_value='false', description='Launch Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='webots_living_room_world.wbt', description='Webots world'),

        # Nodes
        node_webots,
        turtlebot_driver,
        twist_stamper,
        waiting_nodes,
        shutdown_on_exit
    ])
