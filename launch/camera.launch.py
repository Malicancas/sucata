import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get package directory
    pkg_dir = get_package_share_directory('sucata')
    
    # Include RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_color': 'true',
            'enable_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'color_fps': '30.0',
            'depth_fps': '30.0',
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", use_sim_time, "' == 'false'"])
        )
    )
    
    # YOLO 
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_color': 'true',
                'enable_depth': 'true',
                'color_width': '640',
                'color_height': '480',
                'depth_width': '640',
                'depth_height': '480',
                'color_fps': '30.0',
                'depth_fps': '30.0',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", use_sim_time, "' == 'false'"])
            )
        ),
        
        yolo_node,
    ])