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

def generate_webots_launch_description(start_immediately_nodes, start_after_webots_init_nodes, controller_remappings):

    # ARGUMENTS
    world = LaunchConfiguration('world', default='living_room.wbt')

    proto_robot_name = "IsfrFullRobot"

    # PATHS
    isfr_bot_webots_package = get_package_share_directory('isfr_bot_webots')
    isfr_bot_description_package = get_package_share_directory('isfr_bot_description')

    urdf_file = os.path.join(isfr_bot_description_package,'urdf','IsfrFullRobot.urdf')
    ros2_control_file = os.path.join(isfr_bot_webots_package, 'controllers', 'ros2_control.yml')

    # NODES:
    # launches webots
    webotsLauncherNode = WebotsLauncher(
        world=PathJoinSubstitution([isfr_bot_webots_package, 'worlds', world]),
        mode='realtime',
        ros2_supervisor=True,
    )
    
    # Webots main robot controller
    webotsControllerNode = WebotsController(
        robot_name=proto_robot_name,
        parameters=[
            {'robot_description': urdf_file,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
            ros2_control_file
        ],
        remappings=controller_remappings,
        respawn=False
    )

    # Handles spawning the *SpawnerNode nodes after the controller is online
    waitingNodes = WaitForControllerConnection(
        target_driver=webotsControllerNode,
        nodes_to_start=start_after_webots_init_nodes,
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
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='living_room.wbt'),
        webotsLauncherNode,
        webotsLauncherNode._supervisor,
        rspNode,
        footprintPublisherNode,
    ] + start_immediately_nodes + [
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
