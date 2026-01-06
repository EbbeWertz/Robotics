import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('isfr_bot_bringup')
    pkg_desc = get_package_share_directory('isfr_bot_description')

    # Paths
    world_file = os.path.join(pkg_bringup, 'worlds', '/webots_living_room_world.wbt')
    ros2_control_file = os.path.join(pkg_bringup, 'config', 'ros2_control.yaml')

    # URDF paths
    turtlebot_urdf = os.path.join(pkg_desc, 'urdf', 'turtlebot3_waffle.urdf')
    manipulator_urdf = os.path.join(pkg_desc, 'urdf', 'openmanipulator_x.urdf')

    # Option 1: use only TurtleBot3 URDF
    robot_description_file = turtlebot_urdf
    # Option 2: merge URDFs into one if you want combined robot
    # robot_description_file = merge_urdfs(turtlebot_urdf, manipulator_urdf)

    return LaunchDescription([

        # Launch Webots
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            parameters=[{'world': world_file, 'use_sim_time': True}],
        ),

        # Publish robot_description so ros2_control_node can start
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(robot_description_file).read()}],
        ),

        # ROS 2 control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[ros2_control_file, {'use_sim_time': True}],
        ),

        # Spawn controllers
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
