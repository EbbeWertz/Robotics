import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    webots_pkg = get_package_share_directory('isfr_bot_webots')
    description_pkg = get_package_share_directory('isfr_bot_description')

    # --- Include Webots simulator ---
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([webots_pkg, 'launch', 'webots.launch.py'])
        )
    )

    urdf_file = os.path.join(
        description_pkg,
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    robot_description = ParameterValue(
        open(urdf_file).read(),
        value_type=str
    )

    # --- Robot State Publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    diffdrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller"],
        output="screen",
    )

    # Spawn joint state broadcaster
    jointstate_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    return LaunchDescription([
        webots_launch,
        # rsp_node,
        # diffdrive_spawner,
        # jointstate_spawner
    ])
