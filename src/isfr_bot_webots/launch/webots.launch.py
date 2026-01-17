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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # ARGUMENTS
    world = LaunchConfiguration('world', default='living_room.wbt')
    mode = LaunchConfiguration('mode', default='realtime')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_slam = LaunchConfiguration("create_map", default=False)

    proto_robot_name = "TurtleBot3Waffle"

    # PATHS
    isfr_bot_webots_package = get_package_share_directory('isfr_bot_webots')
    isfr_bot_description_package = get_package_share_directory('isfr_bot_description')
    urdf_file = os.path.join(isfr_bot_description_package,'urdf','IsfrFullRobot.urdf')
    ros2_control_file = os.path.join(isfr_bot_webots_package, 'controllers', 'ros2_control.yml')
    slam_config_file = os.path.join(isfr_bot_webots_package,'config','slam_toolbox.yml')
    localisation_config_file = os.path.join(isfr_bot_webots_package, 'config', 'localisation.yml')
    rvizConfigFile = os.path.join(isfr_bot_webots_package, 'config', 'mapmaking.rviz')
    cartographer_config_dir = os.path.join(isfr_bot_webots_package, 'config')
    cartographer_config_file = "cartographer.lua"


    # NODES:
    # launches webots
    webotsLauncherNode = WebotsLauncher(
        world=PathJoinSubstitution([isfr_bot_webots_package, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )
    # Makes topics for joint states
    # WEBOTS CONTROLLER MAKES ITS OWN RSP --> this one commented = not used
    # MAAR FOR SOME REASON KAN JE DEZE NIET WEGLATEN???
    rspNode = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # robot description = empty cuz webotsController node has its own RSP
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
    )
    # makes topics for the robot footprint
    footprintPublisherNode = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
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
    # twistStamperNode = Node(
    #     package='isfr_bot_webots',
    #     executable='twist_stamper',
    #     output='screen'
    # )
    # Webots main robot controller
    webotsControllerNode = WebotsController(
        robot_name=proto_robot_name,
        parameters=[
            {'robot_description': urdf_file,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_file
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        ],
        respawn=False
    )

    # Uses SLAM to create a map    
    # slamToolboxNode = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[slam_config_file],
    #     condition=launch.conditions.IfCondition(use_slam)
    # )
    # turtlebot_slam = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
    #         launch_arguments=[
    #             ('use_sim_time', use_sim_time),
    #         ],
    #         condition=launch.conditions.IfCondition(use_slam))
    cartographerNode = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_file,
        ],
        condition=launch.conditions.IfCondition(use_slam)
    )

    cartographerOccupancyNode = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',  # adjust map resolution as needed
        ],
        condition=launch.conditions.IfCondition(use_slam)
    )

    # slamLifecycleManagerNode = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_slam',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'node_names': ['slam_toolbox']
    #     }],
    #     condition=launch.conditions.IfCondition(use_slam)
    # )
    # rvizNode = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rvizConfigFile],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     condition=launch.conditions.IfCondition(use_slam)
    # )

    # localisationNode = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[localisation_config_file],
    # )

    # Handles spawning the *SpawnerNode nodes after the controller is online
    waitingNodes = WaitForControllerConnection(
        target_driver=webotsControllerNode,
        nodes_to_start=[diffdriveControllerSpawnerNode, jointStateBroadcasterSpawnerNode, cartographerNode, cartographerOccupancyNode],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='living_room.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('create_map', default_value='false'),
        webotsLauncherNode,
        webotsLauncherNode._supervisor,
        rspNode,
        footprintPublisherNode,
        # twistStamperNode,
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
