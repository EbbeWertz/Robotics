from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isfr_bot_vision',
            executable='detector',
            name='yolo_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
            # HIER gebeurt de koppeling tussen Webots en YOLO
            remappings=[
                ('/image_raw', '/camera/image_raw'),       # Pas aan als Webots topic anders heet
                ('/camera_info', '/camera/camera_info')
            ]
        )
    ])