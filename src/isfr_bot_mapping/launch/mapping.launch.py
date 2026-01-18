import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # PATHS
    slam_config_file = os.path.join(get_package_share_directory('isfr_bot_mapping'),'config','slam_toolbox.yml')


    # NODES:
    # Uses SLAM to create a map    
    slamToolboxNode = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file],
    )
    slamLifecycleManagerNode = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }],
    )

    return LaunchDescription([
        slamToolboxNode,
        slamLifecycleManagerNode,
    ])
