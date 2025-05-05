import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='sucata' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Caminho para o mundo do Gazebo
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
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
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Executa o spawner do ros_gz_sim para o robô
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'sucata', '-z', '0.1'],
        output='screen'
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

    teleop = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    output='screen',
    prefix='xterm -e',  # Opcional para terminal dedicado
    parameters=[{'stamped': True}],
    remappings=[('cmd_vel', 'diff_cont/cmd_vel')]
)

    # Bridge de tópicos
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )
        # Lança o RViz com um ficheiro de configuração

    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'andar_bot.rviz')
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
            parameters=[{'use_sim_time': 'false'}, 'src/config/amcl.yaml'],
            remappings=[('/scan', '/laser_scan')]  # Remap o tópico do laser se necessário
    )

    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
    ])