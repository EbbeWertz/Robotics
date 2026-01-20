import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    depth_sentry_node = Node(
            package='isfr_bot_security',
            executable='depth_sentry',
            name='depth_security_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}],
            # Since we hardcoded the topic in the python script to 
            # '/isfr/camera_sensor/depth/image', remapping is optional here.
            # But if you want to change it later without touching code, use this:
            remappings=[
                # ('/isfr/camera_sensor/depth/image', '/another/topic/name')
            ]
        )
    
    patrol_node = Node(
        package='isfr_bot_security',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # Remap /cmd_vel if your robot listens on a specific controller topic
        # remappings=[('/cmd_vel', '/diffdrive_controller/cmd_vel')] 
    )

    return LaunchDescription([
        depth_sentry_node,
        patrol_node
    ])