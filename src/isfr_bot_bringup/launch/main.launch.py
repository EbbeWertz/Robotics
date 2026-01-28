import os
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from isfr_bot_webots.launcher_scaffold import generate_webots_launch_description


############################################################
############################################################
###
###     Deze launcher launcht alles voor de fetch skill:
###
############################################################
############################################################

def generate_launch_description():

    rvizConfigFile = os.path.join(get_package_share_directory('isfr_fetch_skill'), 'rviz', 'visuals_views.rviz')
    map_file = os.path.join(get_package_share_directory('isfr_bot_nav'),'config', 'map.yaml')

    webots_controllers_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_webots'), 'launch', 'controllers.launch.py')
        )
    )

    vision_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_vision'), 'launch', 'yolo.launch.py')
        ),
        launch_arguments=[
            ('map', map_file),
        ]
    )
    
    nav_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_nav'), 'launch', 'nav.launch.py')
        )
    )
    
    grip_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_fetch_skill'), 'launch', 'grip_debug.launch.py')
        )
    )

    arm_ik_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_manipulation'), 'launch', 'ik.launch.py')
        )
    )

    knowledge_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_knowledge'), 'launch', 'knowledge_map.launch.py')
        ),
        launch_arguments=[
            ('map', map_file),
        ]
    )

    fetch_skill_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_fetch_skill'), 'launch', 'fetch_skill.launch.py')
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
    
    return generate_webots_launch_description(
        start_immediately_nodes = [],
        # Voeg hier 'knowledge_launcher' toe aan de lijst
        start_after_webots_init_nodes = [webots_controllers_launcher, vision_launcher, nav_launcher, rvizNode, knowledge_launcher, grip_launcher, arm_ik_launcher, fetch_skill_launcher],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped')]
    )