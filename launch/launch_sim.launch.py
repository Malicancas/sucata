import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Nome do pacote que contém o lançamento do robot_state_publisher
    package_name = 'my_bot'

    # Inclui o lançamento do robot_state_publisher com sim time ativado
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Inclui o lançamento do Gazebo Harmonic (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'verbose': 'true'}.items()  # Adiciona o modo verbose para debug, se necessário
    )

    # Define o nó para spawnar a entidade no Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Retorna a descrição de lançamento com todos os componentes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
