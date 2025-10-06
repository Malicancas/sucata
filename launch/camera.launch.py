import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get package directory
    pkg_dir = get_package_share_directory('sucata')
    
    # Camera launch (RealSense)
    realsense_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'color_fps': 30.0,
            'depth_fps': 30.0,
        }]
    )
    
    # YOLO processing node
    yolo_node = Node(
        package='sucata',
        executable='yolo_camera_node.py',
        name='yolo_camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Nodes
        realsense_camera,
        yolo_node,
    ])