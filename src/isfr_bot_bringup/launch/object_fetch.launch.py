import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from isfr_bot_webots.launcher_scaffold import generate_webots_launch_description


############################################################
############################################################
###
###     Deze launcher launcht ...:
###
############################################################
############################################################

def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('isfr_bot_nav'),'config', 'map.yaml')

    webots_controllers_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_webots'), 'launch', 'controllers.launch.py')
        )
    )
    
    nav_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_nav'), 'launch', 'nav.launch.py')
        )
    )

    fetch_skill_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_fetch_skill'), 'launch', 'fetch_skill.launch.py')
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
    
    knowledge_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_knowledge'), 'launch', 'knowledge_map.launch.py')
        ),
        launch_arguments=[
            ('map', map_file),
        ]
    )

    return generate_webots_launch_description(
        start_immediately_nodes = [],
        start_after_webots_init_nodes = [webots_controllers_launcher, nav_launcher, fetch_skill_launcher, vision_launcher, knowledge_launcher],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped')]
    )
    

