import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nó da câmera
        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'width': 320,
                'height': 240,
                'format': 'YUYV',
                'camera': 0,
                'frame_id': 'camera_link_optical'
            }]
        ),
        
    ])
