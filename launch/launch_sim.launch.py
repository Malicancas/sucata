import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    package_name='sucata'

    gazebo_params_file = os.path.join(package_name, "config/gazebo_params.yaml")

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )
    # Caminho para o mundo do Gazebo
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'mundoArUco.sdf')
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Inclui o lançamento do Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true', "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
arguments=['-topic', 'robot_description', '-name', 'sucata', '-x', '-1', '-y', '-1', '-z', '0.1'],        output='screen'
    )

    # Spawna os controladores
    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_broad"],
    )


    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'map.rviz')
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
    amcl=Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['src/config/amcl.yaml'],
    )

    slam_tool = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch/online_async_launch.py'
        )
    ]),
    launch_arguments={
        'use_sim_time': 'true',
        'slam_params_file': os.path.join(  
            get_package_share_directory(package_name),
            'config/mapper_params_online_async.yaml'
        )
    }.items()
)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        remappings=[("/odometry/filtered", "/odom")],
        parameters=[os.path.join(get_package_share_directory('sucata'), 'config', 'ekf.yaml')]
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            #'controller_frequency': '500.0',
            'params_file': os.path.join(
                get_package_share_directory('sucata'),
                'config/navigation.yaml'
            )
        }.items()
    )
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')],

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
        amcl,
        rviz_node,
        slam_tool,
        ekf_node,
        navigation_launch,
        twist_mux,
    ])