import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'sucata'

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
            )
        ])
    )

    default_world = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'armazem.sdf'
    )
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true',
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'sucata', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'map.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    amcl_param_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'amcl.yaml'
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_param_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    slam_tool = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                get_package_share_directory(package_name),
                'config', 'mapper_params_online_async.yaml'
            )
        }.items()
    )

    ekf_param_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        remappings=[("/odometry/filtered", "/odom")],
        parameters=[ekf_param_file]
    )

    navigation_param_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'navigation.yaml'
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'enable_stamped_cmd_vel': 'true',
            'params_file': navigation_param_file
        }.items()
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')],
    )

    return LaunchDescription([
        rsp,
        joystick,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        # amcl,  # podes descomentar se quiseres usar AMCL
        rviz_node,
        slam_tool,
        ekf_node,
        navigation_launch,
        twist_mux,
    ])
