#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
import launch
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    # Arguments (kan je toevoegen aan de launch command)
    launch_webots = LaunchConfiguration('launch_webots', default='true')
    launch_rqt = LaunchConfiguration('launch_rqt', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world', default='webots_living_room_world.wbt')

    # Package paths
    pkg_bringup = get_package_share_directory('isfr_bot_bringup')
    pkg_desc = get_package_share_directory('isfr_bot_description')
    # File paths
    world_file = PathJoinSubstitution([pkg_bringup, 'worlds', world_file_name])
    urdf_file = os.path.join(pkg_desc, 'urdf', 'turtlebot3_waffle.urdf')
    

    # Webots launcher
    node_webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True
    )

    # Robot state publisher
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                open(urdf_file).read(),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # Optional RQT GUI
    node_rqt_gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen'
    )

    # Event handler: shutdown all nodes when Webots exits
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=node_webots,
            on_exit=[EmitEvent(event=launch.events.Shutdown())]
        )
    )

    # Build launch description
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('launch_webots', default_value='true', description='Whether to start Webots simulation'),
        DeclareLaunchArgument('launch_rqt', default_value='false', description='Whether to start RQT GUI'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='webots_living_room_world.wbt', description='Name of the Webots world file'),

        # Nodes om altijd te runnen:
        node_rsp,

        # Nodes om optioneel te runnen:
        launch.actions.GroupAction(
            actions=[node_webots, shutdown_on_exit],
            condition=launch.conditions.IfCondition(launch_webots)
        ),
        launch.actions.GroupAction(
            actions=[node_rqt_gui],
            condition=launch.conditions.IfCondition(launch_rqt)
        ),
    ])
