import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Path naar je bestaande nav package
    nav_pkg = get_package_share_directory('isfr_bot_nav')

    # 3. Start jouw nieuwe Smart Fetcher node
    fetcher_node = Node(
        package='isfr_fetch_skill',
        executable='smart_fetcher',
        name='smart_fetcher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        fetcher_node
    ])