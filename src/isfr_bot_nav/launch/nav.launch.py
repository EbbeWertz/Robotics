import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # PATHS
    nav2_params_file = os.path.join(get_package_share_directory('isfr_bot_nav'),'config','nav2_params.yml')
    map_file = os.path.join(get_package_share_directory('isfr_bot_nav'),'config', 'map.yaml')

    os.environ['TURTLEBOT3_MODEL'] = 'waffle'
    # NODES:
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'),'launch','bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'params_file': nav2_params_file,
    #         'map': map_file
    #     }.items()
    # )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
        launch_arguments=[
            ('map', map_file),
            ('params_file', nav2_params_file),
            ('use_sim_time', 'true'),
            ('autostart', 'true'),
        ]
    )

    return LaunchDescription([
        nav2_launch
    ])
