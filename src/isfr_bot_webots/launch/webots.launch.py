from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():

    # Launch arguments
    world = LaunchConfiguration('world', default='living_room.wbt')
    mode = LaunchConfiguration('mode', default='realtime')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    package_dir = get_package_share_directory('isfr_bot_webots')
    description_dir = get_package_share_directory('isfr_bot_description')

    # Webots simulator
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # Robot controller for your composite robot
    robot_driver = WebotsController(
        robot_name='IsfrFullRobot',
        parameters=[{
            'robot_description': PathJoinSubstitution([description_dir, 'urdf', 'IsfrFullRobot.urdf']),
            'use_sim_time': use_sim_time
        }],
        respawn=True
    )

    # Optional: custom utility node
    twist_stamper = Node(
        package='isfr_bot_webots',
        executable='twist_stamper',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='living_room.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        webots,
        webots._supervisor,
        robot_driver,
        twist_stamper
    ])
