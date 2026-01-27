import os
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from isfr_bot_webots.launcher_scaffold import generate_webots_launch_description


############################################################
############################################################
###
###     Deze launcher launcht webots met knowledge map:
###
############################################################
############################################################

def generate_launch_description():

    rvizConfigFile = os.path.join(get_package_share_directory('isfr_bot_vision'), 'rviz', 'debug_cameras.rviz')

    webots_controllers_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_webots'), 'launch', 'controllers.launch.py')
        )
    )

    vision_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_vision'), 'launch', 'yolo.launch.py')
        )
    )
    
    nav_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_nav'), 'launch', 'nav.launch.py')
        )
    )
    
    rvizNode = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigFile],
        parameters=[{'use_sim_time': True}],
    )
    
    knowledge_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_knowledge'), 'launch', 'knowledge_map.launch.py')
        )
    )
    
    return generate_webots_launch_description(
        start_immediately_nodes = [],
        # Voeg hier 'knowledge_launcher' toe aan de lijst
        start_after_webots_init_nodes = [webots_controllers_launcher, vision_launcher, nav_launcher, rvizNode, knowledge_launcher],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped')]
    )