from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # haal de config files uit deze isfr_bot_nav pacakge
    pkg_dir = get_package_share_directory('isfr_bot_nav')
    map_file = os.path.join(pkg_dir, 'resource/map.yaml')
    params_file = os.path.join(pkg_dir, 'resource/nav2_params.yaml')

    # launch nav via de installed turtlebot3_navigation2 package
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file
            }.items()
        )
    ])