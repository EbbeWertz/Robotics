import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # ROS2 controller spawner Nodes:
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    # controlls driving around with 2 wheels
    diffdriveControllerSpawnerNode = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    # controls broadcasting all sensor info
    jointStateBroadcasterSpawnerNode = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    groundTruthOdomNode = Node(
        package='isfr_bot_webots',
        executable='ground_truth_odom',
        name='ground_truth_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
        }],
    )


    return LaunchDescription([
        diffdriveControllerSpawnerNode,
        jointStateBroadcasterSpawnerNode,
        groundTruthOdomNode,
    ])
