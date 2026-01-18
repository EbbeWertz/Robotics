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

    webots_controllers_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isfr_bot_webots'), 'launch', 'controllers.launch.py')
        )
    )

    _andere_launcher_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # TODO: _package_, _launchfile_
            os.path.join(get_package_share_directory('_package_'), 'launch', '_launchfile_.launch.py')
        )
    )

    return generate_webots_launch_description(
        start_immediately_nodes = [],
        start_after_webots_init_nodes = [webots_controllers_launcher, _andere_launcher_],
        controller_remappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel')]
    )
    

