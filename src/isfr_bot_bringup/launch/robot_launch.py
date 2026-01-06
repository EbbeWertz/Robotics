import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the bringup package
    pkg_dir = get_package_share_directory('isfr_bot_bringup')

    # Webots world file
    world_file = os.path.join(pkg_dir, 'worlds', 'my_custom_world.wbt')

    # ROS 2 control YAML
    ros2_control_file = os.path.join(pkg_dir, 'config', 'ros2_control.yaml')

    return LaunchDescription([
        # Launch Webots with your world
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            parameters=[{
                'world': world_file,
                'use_sim_time': True
            }],
        ),

        # Start ROS 2 control node with controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[ros2_control_file, {'use_sim_time': True}],
        ),

        # Spawn controllers automatically
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diffdrive_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
        ),
    ])
