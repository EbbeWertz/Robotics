from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    yolo_node = Node(
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
    
    object_analyser_node = Node(
            package='isfr_bot_vision',
            executable='object_analyser',
            name='object_analyser',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )    
    
    # marker_node = Node(
    #         package='isfr_bot_vision',
    #         executable='marker_publisher',
    #         name='vision_marker_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': True}],
    #     )

    return LaunchDescription([
        yolo_node, 
        object_analyser_node,
    ])