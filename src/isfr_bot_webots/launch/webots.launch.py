import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # ARGUMENTS
    world = LaunchConfiguration('world', default='living_room.wbt')
    mode = LaunchConfiguration('mode', default='realtime')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    proto_robot_name = "TurtleBot3Waffle"

    # PATHS
    isfr_bot_webots_package = get_package_share_directory('isfr_bot_webots')
    isfr_bot_description_package = get_package_share_directory('isfr_bot_description')
    urdf_file = os.path.join(isfr_bot_description_package,'urdf','IsfrFullRobot.urdf')
    ros2_control_file = os.path.join(isfr_bot_webots_package, 'resource', 'ros2_control.yaml')

    urdf_robot_content = open(urdf_file).read()

    # NODES:
    # launches webots
    webotsLauncherNode = WebotsLauncher(
        world=PathJoinSubstitution([isfr_bot_webots_package, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )
    # Makes topics for joint states
    # WEBOTS CONTROLLER MAKES ITS OWN RSP --> this one commented = not used
    # rspNode = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     # robot description = empty cuz webotsController node has its own RSP
    #     parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
    # )
    # makes topics for the robot footprint
    footprintPublisherNode = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
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
    # Twist stamper to put timestamps on twist messages:
    # (defined in custom python file in this package)
    twistStamperNode = Node(
        package='isfr_bot_webots',
        executable='twist_stamper',
        output='screen'
    )
    # Webots main robot controller
    webotsControllerNode = WebotsController(
        robot_name=proto_robot_name,
        parameters=[
            {'robot_description': urdf_robot_content,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_file
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped'),
            ('/diffdrive_controller/odom', '/odom')
        ],
        respawn=True
    )

    # Handles spawning the *SpawnerNode nodes after the controller is online
    waitingNodes = WaitForControllerConnection(
        target_driver=webotsControllerNode,
        nodes_to_start=[jointStateBroadcasterSpawnerNode, diffdriveControllerSpawnerNode]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='living_room.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        webotsLauncherNode,
        webotsLauncherNode._supervisor,
        # rspNode,
        footprintPublisherNode,
        twistStamperNode,
        webotsControllerNode,
        waitingNodes,

        # kill all ROS nodes if WebotsLauncherNode is exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webotsLauncherNode,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])
