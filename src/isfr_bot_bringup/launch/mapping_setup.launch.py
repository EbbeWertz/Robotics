import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from isfr_bot_webots.launcher_scaffold import generate_webots_launch_description


############################################################
############################################################
###
###     Deze launcher launcht alle setup voor mapping:
###     - Webots controllers
###     - Mapping nodes
###     - rviz visualisatie, geconfigureerd voor mapping
###
############################################################
############################################################

def generate_launch_description():

    # PATHS
    rvizConfigFile = os.path.join(get_package_share_directory('isfr_bot_mapping'), 'config', 'gui.rviz')

    # NODES
    webots_controllers_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_webots'), 'launch', 'controllers.launch.py')
        )
    )
    mapping_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_mapping'), 'launch', 'mapping.launch.py')
        )
    )
    # opent een rviz window om te visualiseren
    rvizNode = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigFile],
        parameters=[{'use_sim_time': True}],
    )

    return generate_webots_launch_description(
        start_immediately_nodes = [],
        start_after_webots_init_nodes = [webots_controllers_launcher, mapping_launcher, rvizNode],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped')]
    )
    

