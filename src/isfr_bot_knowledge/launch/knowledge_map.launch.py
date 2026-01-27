import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
        
    knowledge_node = Node(
            package='isfr_bot_knowledge',
            executable='object_visualizer',
            name='object_visualizer',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    
    knowledge_map_node = Node(
        package='isfr_bot_knowledge',
        executable='knowledge_map',
        name='knowledge_map',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        knowledge_node,
        knowledge_map_node,
    ])