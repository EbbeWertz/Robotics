import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # NODES:
    ikNode = Node(
            package='isfr_bot_manipulation',
            executable='arm_ik',
            name='arm_inverse_kinematics',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

    return LaunchDescription([
        ikNode
    ])
