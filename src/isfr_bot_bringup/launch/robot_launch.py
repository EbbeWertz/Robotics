import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # maak path naar de webots world file in de bringup package
    pkg_dir = get_package_share_directory('isfr_bot_bringup')
    world_file = os.path.join(pkg_dir, 'worlds/my_custom_world.wbt')

    # launch de world in webots via de webots_ros2_driver
    return LaunchDescription([
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            parameters=[{'world': world_file}],
        )
    ])