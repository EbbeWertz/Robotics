import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from isfr_bot_webots.launcher_scaffold import generate_webots_launch_description
from launch_ros.actions import Node


############################################################
############################################################
###
###     Deze launcher launcht Webots met Navigation:
###
############################################################
############################################################

def generate_launch_description():

    # PATHS
    turtlebot_rviz_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'),'rviz','tb3_navigation2.rviz')

    # Nodes
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
    # rviz_launcher = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', turtlebot_rviz_file],
    #     parameters=[{'use_sim_time': True}],
    # )

    return generate_webots_launch_description(
        start_immediately_nodes = [],
        start_after_webots_init_nodes = [webots_controllers_launcher, nav_launcher],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel_stamped')]
    )
    

