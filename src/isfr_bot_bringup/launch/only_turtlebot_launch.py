import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():

    pkg_bringup = get_package_share_directory('isfr_bot_bringup')
    pkg_desc = get_package_share_directory('isfr_bot_description')

    world_file = os.path.join(
        pkg_bringup,
        'worlds',
        'webots_living_room_world.wbt'
    )

    urdf_file = os.path.join(
        pkg_desc,
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    robot_description = ParameterValue(
        open(urdf_file).read(),
        value_type=str
    )

    webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        webots,
        robot_state_publisher
    ])
