import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Definimos las rutas
    pkg_share = get_package_share_directory('proyecto_modula')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # RUTA DE TU MAPA (Asegúrate de que el nombre coincida con el que guardaste)
    map_file_path = os.path.join(pkg_share, 'maps', 'mapa_simulacion.yaml')

    return LaunchDescription([

        # 1. LiDAR (Mantenerlo para detectar obstáculos en tiempo real)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py'))
        ),

        # 2. Master Controller (Motores)
        Node(
            package='proyecto_modula',
            executable='master_controller',
            name='master_controller_node',
            output='screen'
        ),

        # 3. TF Estática
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.19', '0.0', '0.0', '0.0', 'base_link', 'base_laser'],
            output='screen'
        ),

        # 4. NAV2 BRINGUP (Carga el mapa, AMCL para localizarse y el Navigation Stack)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file_path,
                'use_sim_time': 'false', # Cambiar a 'true' si estás en Gazebo
                'params_file': os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
            }.items()
        ),

        # 5. EL CEREBRO DE LA ASPIRADORA (Tu nuevo script corregido)
        Node(
            package='proyecto_modula',
            executable='movimiento_aspirador',
            name='cerebro_aspiradora',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
